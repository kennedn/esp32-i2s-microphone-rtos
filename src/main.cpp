#include <WiFi.h>

// #include "driver/i2s.h"        // From the newer ESP-IDF/Arduino integration (not legacy i2s.h)
#include <ESP_I2S.h>      
#include <WebServer.h>
#include <wav_header.h>
#include "freertos/stream_buffer.h"

#if defined(BENCHMARK)
#include <AverageFilter.h>
#define BENCHMARK_PRINT_INT 1000
averageFilter<int32_t> streamAvg(10);
uint32_t lastPrint = millis();
#endif

void benchmarkStreamAvgPrint() {
}

#define WSINTERVAL 100

#define PIN_I2S_BCLK     13   // I2S Bit Clock
#define PIN_I2S_WS       15   // I2S Word Select (LRCLK)
#define PIN_I2S_SD       12   // I2S Serial Data (mic data out)

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE_HZ 44100
#define BPS I2S_DATA_BIT_WIDTH_16BIT
#define CH_FMT I2S_STD_SLOT_LEFT

#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN 256 // DMA buffer length in frames

I2SClass i2s;

/* ======== WAV header (for 16-bit mono PCM) ======== */
const uint16_t num_channels = 1; // mono
const uint32_t sample_size = 0xFFFFFFFF; // live stream, so set max size since unknown
const pcm_wav_header_t wav_header = PCM_WAV_HEADER_DEFAULT(sample_size, BPS, SAMPLE_RATE_HZ, num_channels); 

/* ======== Globals ======== */
TaskHandle_t tStreamSetup;
TaskHandle_t tI2SAquire;
TaskHandle_t tClientService;
static QueueHandle_t streamingClients = nullptr;
// ---- HTTP server ----
WebServer server(80);

// Replace with your network credentials

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

static void handleRoot() {
  server.send(200, "text/plain",
              "INMP441 WAV stream available at /stream\r\n"
              "Content-Type: audio/wav\r\n");
}

static void handleAudioStream() {
  // Only accommodate up to 10 clients (WiFiClient queue size)
  if ( !uxQueueSpacesAvailable(streamingClients) ) {
    Serial.println("Max number of WiFi clients reached");
    return;
  }

  // Create a new WiFi Client object for this connection
  WiFiClient* client = new WiFiClient();
  if ( client == NULL ) {
    Serial.println("Can not create new WiFi client - OOM");
    return;
  }

  *client = server.client();

  // Send audio/wav chunked header to client
  client->setTimeout(1);
  client->setNoDelay(true);
  client->print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: audio/wav\r\n"
    "Accept-Ranges: none\r\n"
    "Transfer-Encoding: chunked\r\n"
    "Connection: close\r\n"
    "\r\n"
  );
  client->printf("%X\r\n", (unsigned)sizeof(wav_header));
  client->write((const char*)&wav_header, sizeof(wav_header));
  client->print("\r\n");
  client->flush(); 
  
  // Add the client to the streaming queue
  xQueueSend(streamingClients, (void *) &client, 0);

  if (eTaskGetState(tClientService) == eSuspended) {
    vTaskResume(tClientService);
  }
  Serial.println("Client connected");
}

static void taskClientService(void *pv) {
  const size_t BYTES_PER_MS = (SAMPLE_RATE_HZ * (BPS / 8)) / 1000;
  const size_t BYTES_BUFFER = BYTES_PER_MS * 20; // ~20ms of audio
  std::vector<uint8_t> localBuf(BYTES_BUFFER);
  // std::vector<uint8_t> transBuf(localBuf.size() / 4 * 3);
  uint32_t streamStart = 0;
  for (;;) {
#if defined(BENCHMARK)
    streamStart = micros();
#endif
    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    // No clients, suspend this task
    if ( !activeClients ) {
      vTaskSuspend(NULL);
      continue;
    }
    size_t recievedBytes = i2s.readBytes((char*)localBuf.data(), localBuf.size());

    // int32_t *w = (int32_t*)localBuf.data();
    // Serial.printf("I2S first words: %08X %08X %08X %08X\n", w[0], w[1], w[2], w[3]);

    // // Convert 32-bit samples to 24-bit (drop lowest byte)
    // for (size_t i = 0, j = 0; i + 3 < recievedBytes; i += 4, j += 3) {
    //   transBuf[j]   = localBuf[i+1];
    //   transBuf[j+1] = localBuf[i+2];
    //   transBuf[j+2] = localBuf[i+3];
    // }
    
    WiFiClient *client;
    for (int i=0; i < activeClients; i++) {
      xQueueReceive(streamingClients, (void*)&client, 0);

      if (!client->connected()) {
        Serial.println("Client disconnected");
        client->stop();
        delete client;
        continue;
      }
   
      client->printf("%X\r\n", localBuf.size());
      client->write(localBuf.data(), localBuf.size());
      client->print("\r\n");

      xQueueSend(streamingClients, (void *) &client, 0);
    }
#if defined(BENCHMARK)
    streamAvg.value(micros() - streamStart);
#endif
  }
}

// Legacy I2S acquisition task, not used in current implementation
/*
void taskI2SAquire(void *pvParameters) {
  // Chunk I2S data into ~20ms frames to send to stream buffer
  const size_t BYTES_PER_MS = (SAMPLE_RATE_HZ * (BPS / 8)) / 1000;
  const size_t BYTES_BUFFER = BYTES_PER_MS * 20;

  i2s_event_t event;
  std::vector<uint8_t> localBuf(BYTES_BUFFER);
  size_t burstBytes = 0;

  for (;;) {
    if (xQueueReceive(i2s_evtq, &event, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (event.type != I2S_EVENT_RX_DONE) {
      continue;
    }

    size_t bytesRead = 0;
    esp_err_t ok = i2s_read(I2S_PORT, localBuf.data(), localBuf.size(), &bytesRead, portMAX_DELAY);
    if (ok != ESP_OK) {
      Serial.printf("i2s_read error: %d\n", ok);
      continue;
    }
    
    xStreamBufferSend(audio_sb, localBuf.data(), bytesRead, portMAX_DELAY);
  }
}
*/

void taskStreamSetup(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  streamingClients = xQueueCreate( 10, sizeof(WiFiClient*) );
  // HTTP endpoints
  server.onNotFound(handleRoot);
  server.on("/stream", HTTP_GET, handleAudioStream);
  server.begin();

  Serial.println("[HTTP] Server started");

  // Start client service task (handles I2S and streaming)
  xTaskCreatePinnedToCore(taskClientService, "taskClientService", 4 * 1024, NULL, tskIDLE_PRIORITY + 2, &tClientService, 1);

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();
    
#if defined(BENCHMARK)
    if (millis() - lastPrint > BENCHMARK_PRINT_INT) {
        lastPrint = millis();
        Serial.printf("streamCB: stream avg=%d us\n", streamAvg.currentValue());
    }
#endif

    // Let other tasks run and then pause
    if ( xTaskDelayUntil(&xLastWakeTime, xFrequency) != pdTRUE ) taskYIELD();
  }
}

// ---- I2S setup using ESP_I2S.h ----
static void I2SSetup() {
  // i2s_config_t cfg = {
  //   .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  //   .sample_rate = SAMPLE_RATE_HZ,
  //   .bits_per_sample = BPS,
  //   .channel_format = CH_FMT,
  //   .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  //   .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  //   .dma_buf_count = DMA_BUF_COUNT,
  //   .dma_buf_len = DMA_BUF_LEN,
  // };

  // i2s_pin_config_t pins = {
  //   .bck_io_num   = PIN_I2S_BCLK,
  //   .ws_io_num    = PIN_I2S_WS,
  //   .data_out_num = I2S_PIN_NO_CHANGE,  // RX only
  //   .data_in_num  = PIN_I2S_SD
  // };

  // // Install driver (no event queue used)
  // ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, nullptr));
  // ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
  // ESP_ERROR_CHECK(i2s_start(I2S_PORT));
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, -1, PIN_I2S_SD, -1); // BCLK/SCK, LRCLK/WS, SDOUT, SDIN, MCLK
  bool ok = i2s.begin(I2S_MODE_STD, SAMPLE_RATE_HZ, BPS, I2S_SLOT_MODE_MONO, CH_FMT);
  if (!ok) {
    Serial.println("I2S begin failed");
    return;
  }
}



void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  // Initialize Wi-Fi and connect to AP
  int status = WiFi.begin((char*)ssid, (char*)password);

  Serial.println("[WiFi] Connecting to WiFi");

  delay(5000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("[WiFi] Error connecting to WiFi: ");
    Serial.println(WiFi.status());
  }
  Serial.print("[WiFi] Connected. IP: ");
  Serial.println(WiFi.localIP());

  // ---- I2S ----
  I2SSetup();

  xTaskCreatePinnedToCore(taskStreamSetup, "taskStreamSetup", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &tStreamSetup, 1);

  Serial.printf("setup complete: free heap  : %d\n", ESP.getFreeHeap());

}

void loop() {
  delay(1000);
}