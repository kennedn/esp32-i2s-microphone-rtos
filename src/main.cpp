#include <WiFi.h>

#include "driver/i2s.h"        // From the newer ESP-IDF/Arduino integration (not legacy i2s.h)
#include <WebServer.h>    // tzapu/WiFiManager
#include <wav_header.h>
#include "freertos/stream_buffer.h"

#define WSINTERVAL 100

#define PIN_I2S_BCLK     13   // BCLK
#define PIN_I2S_WS       15   // LRCLK / WS
#define PIN_I2S_SD       12   // SD  (mic data out)

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE_HZ 48000
#define BPS I2S_BITS_PER_SAMPLE_16BIT
#define CH_FMT I2S_CHANNEL_FMT_ONLY_LEFT

#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN 256 // frames per DMA buffer (tune for latency/CPU)

/* Stream buffer (lock-free ring). Size in bytes; must hold a few DMA buffers. */
static const size_t STREAM_CAP_BYTES = 64 * 1024;

/* ======== WAV header (for 16-bit mono PCM) ======== */
const uint16_t num_channels = 1; // mono
const uint32_t sample_size = 0xFFFFFFFF; // live stream, so set max size since unknown
const pcm_wav_header_t wav_header = PCM_WAV_HEADER_DEFAULT(sample_size, BPS, SAMPLE_RATE_HZ, num_channels); 

/* ======== Globals ======== */
TaskHandle_t tStreamSetup;
TaskHandle_t tI2SAquire;
TaskHandle_t tClientService;
static QueueHandle_t i2s_evtq = nullptr;
static QueueHandle_t streamingClients = nullptr;
static StreamBufferHandle_t audio_sb = nullptr;
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
  //  Can only acommodate 10 clients. The limit is a default for WiFi connections
  if ( !uxQueueSpacesAvailable(streamingClients) ) {
    Serial.println("Max number of WiFi clients reached");
    return;
  }

  //  Create a new WiFi Client object to keep track of this one
  WiFiClient* client = new WiFiClient();
  if ( client == NULL ) {
    Serial.println("Can not create new WiFi client - OOM");
    return;
  }

  *client = server.client();

  // Immediately send audio/wav chunked header to client
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
  
  // Push the client to the streaming queue
  xQueueSend(streamingClients, (void *) &client, 0);

  if (eTaskGetState(tClientService) == eSuspended) {
    vTaskResume(tClientService);
  }
  Serial.println("Client connected");
}

static void taskClientService(void *pv) {
  std::vector<uint8_t> chunk(2048);
  for (;;) {
    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    // No clients so we can pause this task.
    if ( !activeClients ) {
      vTaskSuspend(NULL);
      continue;
    }

    size_t recievedBytes = xStreamBufferReceive(audio_sb, chunk.data(), chunk.size(), portMAX_DELAY);
    
    WiFiClient *client;
    for (int i=0; i < activeClients; i++) {
      xQueueReceive(streamingClients, (void*)&client, 0);

      if (!client->connected()) {
        Serial.println("Client disconnected");
        client->stop();
        delete client;
        continue;
      }
   
      client->printf("%X\r\n", recievedBytes);
      client->write(chunk.data(), recievedBytes);
      client->print("\r\n");

      xQueueSend(streamingClients, (void *) &client, 0);
    }
  }
}

void taskI2SAquire(void *pvParameters) {
  // Chunk I2S data into ~20ms frames to send to stream buffer
  const size_t BYTES_PER_MS = (SAMPLE_RATE_HZ * (16 / 8)) / 1000;  // 48k * 2 / 1000 = 96 B/ms
  const size_t BYTES_BUFFER = BYTES_PER_MS * 20;                  // * 20ms = ~1920 bytes

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

void taskStreamSetup(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Reserve ~200ms of audio buffer and trigger in 20ms chunks, this sets the cadence of the xStreamBufferReceive in taskClientService
  const size_t BYTES_PER_MS = (SAMPLE_RATE_HZ * (16 / 8)) / 1000;  // 48k * 2 / 1000 = 96 B/ms
  const size_t AUDIO_FRAME = BYTES_PER_MS * 20;           // * 20ms = ~1920 bytes                    
  const size_t BUFFER_SIZE = BYTES_PER_MS * 200;           // * 200ms = ~19.2k bytes
  audio_sb = xStreamBufferCreate(BUFFER_SIZE, AUDIO_FRAME);

  streamingClients = xQueueCreate( 10, sizeof(WiFiClient*) );
  // ---- HTTP endpoints ----
  server.onNotFound(handleRoot);
  server.on("/stream", HTTP_GET, handleAudioStream);
  server.begin();

  Serial.println("[HTTP] Server started");

  xTaskCreatePinnedToCore(taskI2SAquire, "taskI2SAquire", 4 * 1024, NULL, tskIDLE_PRIORITY + 2, &tI2SAquire, 1);
  xTaskCreatePinnedToCore(taskClientService, "taskClientService", 4 * 1024, NULL, tskIDLE_PRIORITY + 2, &tClientService, 1);

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();

    //  After every server client handling request, we let other tasks run and then pause
    if ( xTaskDelayUntil(&xLastWakeTime, xFrequency) != pdTRUE ) taskYIELD();
  }
}

// ---- I2S setup using ESP_I2S.h ----
static void I2SSetup() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE_HZ,
    .bits_per_sample = BPS,
    .channel_format = CH_FMT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num   = PIN_I2S_BCLK,
    .ws_io_num    = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,  // RX only
    .data_in_num  = PIN_I2S_SD
  };

  // Install driver with event queue so we can react to DMA completion
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, /*queue size*/ 8, &i2s_evtq));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
  ESP_ERROR_CHECK(i2s_start(I2S_PORT));
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