#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <USBMSC.h>
#include <USB.h>
#include "driver/i2s.h"
#include "wav.h"


SPIClass sdSPI(FSPI);

#define UART1_TX_PIN 17
#define UART1_RX_PIN 18


std::vector<String> wavList;


float g_volume = 1.0f;


void commandParserTask(void *parameter);
void playbackTask(void *parameter);



void setupAudioMode() {
  Serial.println("Init SPI SD...");
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  Serial.print("Init SD...");
  if (!SD.begin(SD_CS_PIN, sdSPI)) {
    Serial.println(" ECHEC !");
    while (true) {
      Serial.println("Erreur SD.begin(). Vérifie SCK/MISO/MOSI/CS.");
      delay(2000);
    }
  }
  
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Impossible d'ouvrir /");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();
      String lower = name;
      lower.toLowerCase();

      if (lower.endsWith(".wav")) {
        wavList.push_back(name);
      }
    }
    file = root.openNextFile();
  }

  Serial.printf("Trouvé %u fichier(s) WAV :\n", wavList.size());

  for (size_t i = 0; i < wavList.size(); i++) {
    Serial.printf("  [%u] %s\n", i, wavList[i].c_str());
  }
  g_wavCmdQueue = xQueueCreate(4, sizeof(WavCommand));

  xTaskCreatePinnedToCore(playbackTask,"playbackTask",8192,nullptr,2,nullptr,1);
  xTaskCreatePinnedToCore(commandParserTask,"commandParserTask",4096,nullptr,1,nullptr,0);
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial1.begin(115200,SERIAL_8N1,UART1_RX_PIN,UART1_TX_PIN);

  Serial.println("\n=== WAV Player ESP32-S3 + MAX99357 ===");
  
  Serial.println("Mode Audio (lecteur WAV)");
  setupAudioMode();

  pinMode(AMP_SD_MODE_PIN, OUTPUT);
  digitalWrite(AMP_SD_MODE_PIN, LOW);
}

void loop() {

  vTaskDelay(10000);
}



void playbackTask(void *parameter) {
  (void)parameter;

  for (;;) {
    if (g_wavCmdQueue == NULL) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    WavCommand cmd;
    if (xQueueReceive(g_wavCmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {

      if (cmd.cmd == 'p') {
        Serial.print("CMD PLAY: ");
        Serial.println(cmd.path);
        playWav(cmd.path, false);
      }
      else if (cmd.cmd == 'l') {
        Serial.print("CMD LOOP: ");
        Serial.println(cmd.path);
        playWav(cmd.path, true);
      }
      else if (cmd.cmd == 's') {
        Serial.println("CMD STOP en idle.");
      }
    }
  }
}


void commandParserTask(void *parameter) {
  (void)parameter;

  static char usbBuffer[160];
  static char uartBuffer[160];
  size_t usbIdx = 0;
  size_t uartIdx = 0;

  Serial.println("Command parser task ready");

  for (;;) {

    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (usbIdx > 0) {
          usbBuffer[usbIdx] = '\0';
          usbIdx = 0;

          WavCommand cmd = {};
          cmd.cmd = usbBuffer[0];

          if (cmd.cmd == 's') {
            Serial.println("CMD STOP");
            xQueueSend(g_wavCmdQueue, &cmd, 0);
          }
          else if ((cmd.cmd == 'p' || cmd.cmd == 'l') && usbBuffer[1] == ' ') {
            strncpy(cmd.path, &usbBuffer[2], sizeof(cmd.path) - 1);
            cmd.path[sizeof(cmd.path) - 1] = '\0';

            Serial.printf("CMD %c %s\n", cmd.cmd, cmd.path);
            xQueueSend(g_wavCmdQueue, &cmd, 0);
          }
          else {
            Serial.printf("Commande invalide: %s\n", usbBuffer);
          }
        }
      }
      else if (usbIdx < sizeof(usbBuffer) - 1) {
        usbBuffer[usbIdx++] = c;
      }
    }

    while (Serial1.available()) {
      char c = Serial1.read();

      if (c == '\n' || c == '\r') {
        if (uartIdx > 0) {
          uartBuffer[uartIdx] = '\0';
          uartIdx = 0;

          WavCommand cmd = {};
          cmd.cmd = uartBuffer[0];

          if (cmd.cmd == 's') {
            Serial1.println("CMD STOP");
            xQueueSend(g_wavCmdQueue, &cmd, 0);
          }
          else if ((cmd.cmd == 'p' || cmd.cmd == 'l') && uartBuffer[1] == ' ') {
            strncpy(cmd.path, &uartBuffer[2], sizeof(cmd.path) - 1);
            cmd.path[sizeof(cmd.path) - 1] = '\0';

            Serial1.printf("CMD %c %s\n", cmd.cmd, cmd.path);
            xQueueSend(g_wavCmdQueue, &cmd, 0);
          }
          else {
            Serial1.printf("Commande invalide: %s\n", uartBuffer);
          }
        }
      }
      else if (uartIdx < sizeof(uartBuffer) - 1) {
        uartBuffer[uartIdx++] = c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

