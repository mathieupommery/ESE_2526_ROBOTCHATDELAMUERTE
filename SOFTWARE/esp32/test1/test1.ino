#include <driver/i2s.h>

#define I2S_WS     8    // LRCLK
#define I2S_SCK    6    // BCLK
#define I2S_SD     7    // DOUT du micro

#define SAMPLE_RATE   16000
#define SAMPLES       256

void setup() {
  Serial.begin(115200);
  delay(500);

  // Configuration I2S
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = SAMPLES,
    .use_apll = false
  };

  i2s_pin_config_t pin_cfg = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_cfg);
  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("I2S Microphone Test Ready...");
}

void loop() {
  int32_t raw[ SAMPLES ];
  size_t bytes_read;

  i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);

  if (bytes_read > 0) {
    // Conversion simple : on ne prend que la partie utile
    for (int i = 0; i < SAMPLES; i++) {
      int32_t sample = raw[i] >> 14;   // Normalisation
      Serial.println(sample);          // Pour Serial Plotter
    }
  }
}
