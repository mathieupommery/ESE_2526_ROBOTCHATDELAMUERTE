#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <USBMSC.h>
#include <USB.h>
#include "driver/i2s.h"
#include "wav.h"


#define LONG_PRESS_MS 400

static const uint16_t SECTOR_SIZE = 512;

SPIClass sdSPI(FSPI);   // bus SPI pour la SD sur ESP32-S3 (FSPI)

// =======================
// Paramètres WAV & volume
// =======================
std::vector<String> wavList;

// Volume logiciel (0.0 = muet, 1.0 = plein volume)
float g_volume = 1.0f;   // tu peux régler ça



// =======================
// Prototypes
// =======================
void playbackTask(void *parameter);
void buttonTask(void *parameter);




void setupAudioMode() {
  // Init SPI pour la carte SD
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
}


void setup() {
  Serial.begin(115200);
  delay(500);
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
    // On attend une commande
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
        Serial.println("CMD STOP en idle (ignorée ici).");
      }
    }
  }
}

