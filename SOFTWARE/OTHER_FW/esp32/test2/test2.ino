#include <Arduino.h>
#include <SPI. h>
#include <SD. h>
#include "driver/i2s.h"

#define I2S_BCLK_PIN    2    // BCLK
#define I2S_LRCLK_PIN   1    // LRCLK / WS
#define I2S_DOUT_PIN    4    // DIN du MAX99357
#define AMP_SD_MODE_PIN 5    // SD_MODE (shutdown / gain)

#define SD_MISO_PIN     13
#define SD_MOSI_PIN     11
#define SD_SCK_PIN      12
#define SD_CS_PIN       10

// UART pins for communication
#define UART_RX_PIN     18
#define UART_TX_PIN     17

SPIClass sdSPI(FSPI);   // bus SPI pour la SD sur ESP32-S3 (FSPI)

const uint32_t LOOP_DELAY_MS = 200;
float g_volume = 0.9f;

// Command types
enum CommandType {
  CMD_NONE,
  CMD_LOOP,       // L,<filename>
  CMD_PLAY_ONCE,  // P,<filename>
  CMD_STOP        // S
};

// Command structure
typedef struct {
  CommandType type;
  char filename[64];
} AudioCommand;

// Queue for audio commands
QueueHandle_t audioCommandQueue;
const int QUEUE_SIZE = 10;

// Flag to stop current playback
volatile bool stopPlayback = false;

typedef struct {
  uint16_t audioFormat;   // 1 = PCM
  uint16_t channels;      // 1 = mono, 2 = stéréo
  uint32_t sampleRate;    // ex: 44100
  uint16_t bitsPerSample; // 16 attendu ici
  uint32_t dataSize;      // taille du chunk "data"
} WavHeader;

void playbackTask(void *parameter);
void uartCommandTask(void *parameter);
bool initI2S(uint32_t sampleRate);
bool readWavHeader(File &f, WavHeader &hdr);
int16_t applyVolume(int16_t sample);
void parseUartCommand(String command);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== WAV Player ESP32-S3 + MAX99357 with UART Control ===");

  // Initialize UART for commands (Serial2)
  Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART initialized on Serial2");
  Serial.println("Commands:  L,<filename> | P,<filename> | S");

  pinMode(AMP_SD_MODE_PIN, OUTPUT);
  digitalWrite(AMP_SD_MODE_PIN, LOW);

  Serial.println("Init SPI SD...");
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  Serial.print("Init SD...");
  if (!SD.begin(SD_CS_PIN, sdSPI)) {
    Serial.println(" ECHEC !");
    while (true) {
      Serial.println("Erreur SD. begin(). Vérifie SCK/MISO/MOSI/CS.");
      delay(2000);
    }
  }
  Serial.println(" OK.");

  // Create command queue
  audioCommandQueue = xQueueCreate(QUEUE_SIZE, sizeof(AudioCommand));
  if (audioCommandQueue == NULL) {
    Serial.println("Failed to create command queue!");
    while(1) delay(1000);
  }

  // Create tasks
  xTaskCreatePinnedToCore(playbackTask, "playbackTask", 8192, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(uartCommandTask, "uartCommandTask", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void uartCommandTask(void *parameter) {
  (void)parameter;
  String commandBuffer = "";

  Serial.println("UART Command Task started");

  while (true) {
    while (Serial2.available()) {
      char c = Serial2.read();
      
      if (c == '\n' || c == '\r') {
        if (commandBuffer.length() > 0) {
          parseUartCommand(commandBuffer);
          commandBuffer = "";
        }
      } else {
        commandBuffer += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void parseUartCommand(String command) {
  command.trim();
  Serial.print("Received command: ");
  Serial.println(command);

  AudioCommand cmd;
  cmd.type = CMD_NONE;
  memset(cmd.filename, 0, sizeof(cmd.filename));

  if (command.startsWith("L,")) {
    // Loop command
    cmd.type = CMD_LOOP;
    String filename = command.substring(2);
    filename.trim();
    
    // Ensure filename starts with /
    if (! filename.startsWith("/")) {
      filename = "/" + filename;
    }
    
    strncpy(cmd.filename, filename.c_str(), sizeof(cmd.filename) - 1);
    
    Serial.print("Queuing LOOP command for: ");
    Serial.println(cmd.filename);
    
    if (xQueueSend(audioCommandQueue, &cmd, 0) != pdTRUE) {
      Serial.println("Queue full!  Command dropped.");
    }
  }
  else if (command.startsWith("P,")) {
    // Play once command
    cmd. type = CMD_PLAY_ONCE;
    String filename = command.substring(2);
    filename.trim();
    
    // Ensure filename starts with /
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    
    strncpy(cmd.filename, filename.c_str(), sizeof(cmd.filename) - 1);
    
    Serial.print("Queuing PLAY_ONCE command for: ");
    Serial.println(cmd.filename);
    
    if (xQueueSend(audioCommandQueue, &cmd, 0) != pdTRUE) {
      Serial.println("Queue full! Command dropped.");
    }
  }
  else if (command.equals("S")) {
    // Stop command - clear queue and stop playback
    cmd.type = CMD_STOP;
    
    Serial.println("STOP command received - clearing queue");
    
    // Clear the queue
    xQueueReset(audioCommandQueue);
    
    // Add stop command
    if (xQueueSend(audioCommandQueue, &cmd, 0) != pdTRUE) {
      Serial.println("Failed to send STOP command");
    }
    
    // Signal to stop current playback
    stopPlayback = true;
  }
  else {
    Serial.println("Unknown command format!");
    Serial.println("Valid commands: L,<filename> | P,<filename> | S");
  }
}

int16_t applyVolume(int16_t sample) {
  float s = (float)sample * g_volume;
  if (s > 32767.0f) s = 32767.0f;
  if (s < -32768.0f) s = -32768.0f;
  return (int16_t)s;
}

void playbackTask(void *parameter) {
  (void)parameter;
  AudioCommand currentCmd;
  bool isPlaying = false;
  bool shouldLoop = false;

  Serial.println("Playback Task started - waiting for commands");

  while (true) {
    // Check if we should get a new command
    if (! isPlaying) {
      // Wait for a command from the queue
      if (xQueueReceive(audioCommandQueue, &currentCmd, portMAX_DELAY) == pdTRUE) {
        
        if (currentCmd.type == CMD_STOP) {
          Serial.println("Processing STOP command");
          stopPlayback = false;
          digitalWrite(AMP_SD_MODE_PIN, LOW);
          isPlaying = false;
          continue;
        }
        
        if (currentCmd.type == CMD_LOOP || currentCmd.type == CMD_PLAY_ONCE) {
          shouldLoop = (currentCmd.type == CMD_LOOP);
          isPlaying = true;
          stopPlayback = false;
          
          Serial.print("Starting playback: ");
          Serial.print(currentCmd.filename);
          Serial.println(shouldLoop ? " (LOOP)" : " (ONCE)");
        }
      }
    }

    if (! isPlaying) {
      continue;
    }

    // Open the file
    File wavFile = SD.open(currentCmd.filename);
    if (!wavFile) {
      Serial.print("Cannot open file: ");
      Serial.println(currentCmd.filename);
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    WavHeader hdr;
    if (!readWavHeader(wavFile, hdr)) {
      Serial.println("Invalid WAV header");
      wavFile.close();
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (hdr.bitsPerSample != 16) {
      Serial.println("Only 16 bits supported");
      wavFile.close();
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (hdr.channels != 1 && hdr. channels != 2) {
      Serial.println("Only mono or stereo supported");
      wavFile.close();
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (! initI2S(hdr. sampleRate)) {
      Serial.println("I2S init error");
      wavFile.close();
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    digitalWrite(AMP_SD_MODE_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5));

    const size_t BUF_SIZE = 1024;
    uint8_t buf[BUF_SIZE];
    size_t bytesRead;
    bool fileFinished = false;

    // Playback loop
    while (!stopPlayback && (bytesRead = wavFile.read(buf, BUF_SIZE)) > 0) {
      
      if (hdr.channels == 1) {
        // Mono -> Stereo conversion with volume
        size_t samplesMono = bytesRead / 2;
        static int16_t tempStereo[BUF_SIZE];

        size_t stereoCount = 0;
        int16_t *src = (int16_t *)buf;

        for (size_t i = 0; i < samplesMono; i++) {
          int16_t s = applyVolume(src[i]);
          tempStereo[stereoCount++] = s; // left
          tempStereo[stereoCount++] = s; // right
        }

        size_t bytesToWrite = stereoCount * 2;
        size_t written = 0;
        while (written < bytesToWrite && !stopPlayback) {
          size_t w = 0;
          i2s_write(I2S_NUM_0,
                    ((uint8_t *)tempStereo) + written,
                    bytesToWrite - written,
                    &w,
                    100 / portTICK_PERIOD_MS);
          written += w;
        }

      } else {
        // Stereo with volume
        size_t samplesStereo = bytesRead / 2;
        int16_t *st = (int16_t *)buf;

        for (size_t i = 0; i < samplesStereo; i++) {
          st[i] = applyVolume(st[i]);
        }

        size_t written = 0;
        while (written < bytesRead && !stopPlayback) {
          size_t w = 0;
          i2s_write(I2S_NUM_0,
                    buf + written,
                    bytesRead - written,
                    &w,
                    100 / portTICK_PERIOD_MS);
          written += w;
        }
      }
    }

    wavFile.close();

    if (stopPlayback) {
      Serial.println("Playback stopped by command");
      // Fade out
      for (int i = 0; i < 256; i++) {
        int16_t frame[2] = {0, 0};
        size_t written;
        i2s_write(I2S_NUM_0, frame, sizeof(frame), &written, 100 / portTICK_PERIOD_MS);
      }
      digitalWrite(AMP_SD_MODE_PIN, LOW);
      isPlaying = false;
      stopPlayback = false;
      continue;
    }

    // File finished
    if (shouldLoop) {
      // Loop mode - restart the same file
      Serial.println("Looping.. .");
      vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    } else {
      // Play once mode - finish and get next command
      Serial.println("Playback finished");
      // Fade out
      for (int i = 0; i < 256; i++) {
        int16_t frame[2] = {0, 0};
        size_t written;
        i2s_write(I2S_NUM_0, frame, sizeof(frame), &written, 100 / portTICK_PERIOD_MS);
      }
      digitalWrite(AMP_SD_MODE_PIN, LOW);
      isPlaying = false;
      vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
  }
}

bool initI2S(uint32_t sampleRate) {
  static bool i2sAlreadyInit = false;

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    . use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    . bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    . data_in_num = I2S_PIN_NO_CHANGE
  };

  if (! i2sAlreadyInit) {
    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK) {
      Serial.println("i2s_driver_install failed");
      return false;
    }
    if (i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK) {
      Serial.println("i2s_set_pin failed");
      return false;
    }
    i2sAlreadyInit = true;
  }

  if (i2s_set_sample_rates(I2S_NUM_0, sampleRate) != ESP_OK) {
    Serial.println("i2s_set_sample_rates failed");
    return false;
  }

  return true;
}

bool readWavHeader(File &f, WavHeader &hdr) {
  f.seek(0);

  char riff[4];
  char wave[4];
  uint32_t riffSize;

  if (f.read((uint8_t *)riff, 4) != 4) return false;
  if (f.read((uint8_t *)&riffSize, 4) != 4) return false;
  if (f.read((uint8_t *)wave, 4) != 4) return false;

  if (strncmp(riff, "RIFF", 4) != 0) return false;
  if (strncmp(wave, "WAVE", 4) != 0) return false;

  bool fmtFound  = false;
  bool dataFound = false;

  while (f.available()) {
    char  subId[4];
    uint32_t subSize;

    if (f.read((uint8_t *)subId, 4) != 4) return false;
    if (f.read((uint8_t *)&subSize, 4) != 4) return false;

    if (strncmp(subId, "fmt ", 4) == 0) {
      if (subSize < 16) {
        return false;
      }

      uint8_t fmtBuf[16];
      if (f.read(fmtBuf, 16) != 16) return false;

      hdr.audioFormat   = (uint16_t)(fmtBuf[0] | (fmtBuf[1] << 8));
      hdr.channels      = (uint16_t)(fmtBuf[2] | (fmtBuf[3] << 8));
      hdr.sampleRate    = (uint32_t)(fmtBuf[4] |
                                     (fmtBuf[5] << 8) |
                                     (fmtBuf[6] << 16) |
                                     (fmtBuf[7] << 24));
      hdr.bitsPerSample = (uint16_t)(fmtBuf[14] | (fmtBuf[15] << 8));

      if (subSize > 16) {
        uint32_t toSkip = subSize - 16;
        f.seek(f.position() + toSkip);
      }

      fmtFound = true;
    }
    else if (strncmp(subId, "data", 4) == 0) {
      hdr.dataSize = subSize;
      dataFound = true;
      break;
    }
    else {
      f.seek(f. position() + subSize);
    }
  }

  if (! fmtFound || !dataFound) {
    return false;
  }

  if (hdr.audioFormat != 1) {
    Serial.print("Format audio non PCM, code = ");
    Serial.println(hdr.audioFormat);
    return false;
  }

  return true;
}