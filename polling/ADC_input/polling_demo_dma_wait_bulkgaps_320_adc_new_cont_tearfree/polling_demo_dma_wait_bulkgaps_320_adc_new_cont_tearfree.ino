// continuous_adc_plot_esp32_tearfree.ino
// Modified to use the tear-free display timing framework but WITHOUT double buffering.
// ADC continuous (DMA) -> decimate -> write into shared_out -> main loop calls plotter.startBarPlot at the right time.

#include "BarPlotter.h"
#include <TFT_eSPI.h>

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "driver/adc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
}

#include <SPI.h>

#define SCREEN_UPDATE_RATE_MS 33     // ~30 FPS (informational)

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// ADC continuous settings
#define ADC_SAMPLE_FREQ_HZ    31300UL
#define ADC_READ_BUF_BYTES    (8 * 1024)
#define ADC_FRAME_SIZE        1024
#define ADC_READ_TIMEOUT_MS   50

#define DECIMATE_PER_FRAME    2

// Tearfree display timing settings (from your reference)
float   DISPLAY_REFRESH_RATE = 33.0f;

#ifdef TEARING_DEBUG
uint8_t var1 = 0x1f; // frame 
uint8_t var2 = 0x16; // front porch
uint8_t var3 = 0x3f; // back porch

uint8_t var4 = 0x1f; // frame 
uint8_t var5 = 0x40; // front porch
uint8_t var6 = 0x3f;  // back porch

uint8_t selected = 7;
#endif

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ADC handle
static adc_continuous_handle_t adc_handle = NULL;

// buffer for reading raw ADC bytes from driver
static uint8_t *adc_read_buffer = nullptr;
static volatile bool adc_running = false;

// Shared plot buffer (single buffer, protected by mutex)
static uint16_t shared_out[BAR_COUNT];
static volatile bool bufferReady = false;
SemaphoreHandle_t bufferMutex = NULL;

// Frame timing control (tear-free)
unsigned long lastDisplayTime = 0;
bool oddFrame = false;
bool oddFrameAdjusted = false;

#ifdef TEARING_DEBUG
void serial_tft_helper () {
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input >= '1' && input <= '7') selected = input - '0';
      else if (input == '-') {
        switch (selected) {
          case 1: var1--; break;
          case 2: var2--; break;
          case 3: var3--; break;
          case 4: var4--; break;
          case 5: var5--; break;
          case 6: var6--; break;
          case 7: DISPLAY_REFRESH_RATE -= 1.0; break;
        }
      } else if (input == '=') {
        switch (selected) {
          case 1: var1++; break;
          case 2: var2++; break;
          case 3: var3++; break;
          case 4: var4++; break;
          case 5: var5++; break;
          case 6: var6++; break;
          case 7: DISPLAY_REFRESH_RATE += 1.0; break;
        }
      }
      Serial.print("1 RTNA1 = "); Serial.print(var1,HEX);
      Serial.print(", 2 Fporch1= "); Serial.print(var2,HEX);
      Serial.print(", 3 Bporch1 = "); Serial.print(var3,HEX);
      Serial.print(", 4 RTNA2 = "); Serial.print(var4,HEX);
      Serial.print(", 5 Fporch2 = "); Serial.print(var5,HEX);
      Serial.print(", 6 Bporch2 = "); Serial.print(var6,HEX);
      Serial.print(", 7 VSYNC = "); Serial.print(DISPLAY_REFRESH_RATE,2);
      Serial.print(", selected = "); Serial.println(selected);
    }
}
#endif

// ---------- ADC setup ----------
void setup_adc_continuous()
{
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  adc_continuous_handle_cfg_t handle_cfg = {
      .max_store_buf_size = ADC_READ_BUF_BYTES,
      .conv_frame_size = ADC_FRAME_SIZE,
      .flags = {0}
  };

  esp_err_t r = adc_continuous_new_handle(&handle_cfg, &adc_handle);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_new_handle failed: ");
    Serial.println(r);
    return;
  }

  static adc_digi_pattern_config_t pattern[1];
  pattern[0].atten = ADC_ATTEN_DB_11;
  pattern[0].channel = ADC_CHANNEL_0;
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {
      .pattern_num = 1,
      .adc_pattern = pattern,
      .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
  };

  r = adc_continuous_config(adc_handle, &dig_cfg);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_config failed: ");
    Serial.println(r);
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  adc_read_buffer = (uint8_t*)malloc(ADC_READ_BUF_BYTES);
  if (!adc_read_buffer) {
    Serial.println("adc_read_buffer malloc failed");
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  r = adc_continuous_start(adc_handle);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_start failed: ");
    Serial.println(r);
    free(adc_read_buffer);
    adc_read_buffer = nullptr;
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  adc_running = true;
  Serial.println("ADC continuous started");
}

// ADC reader task: fills ring, decimates, writes into shared_out (mutex-protected)
void adc_reader_task(void *pv)
{
  const size_t ringSize = BAR_COUNT * DECIMATE_PER_FRAME * 2;
  uint16_t *ring = (uint16_t*)malloc(sizeof(uint16_t) * ringSize);
  if (!ring) {
    Serial.println("ring alloc failed");
    vTaskDelete(NULL);
    return;
  }
  size_t ring_pos = 0;

  uint16_t *local_out = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  if (!local_out) {
    Serial.println("out alloc failed");
    free(ring);
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    if (!adc_running) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint32_t out_len = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, adc_read_buffer, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
      continue;
    } else if (ret == ESP_OK && out_len > 0) {
      size_t item_size = sizeof(adc_digi_output_data_t);
      size_t count = out_len / item_size;
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)adc_read_buffer;

      for (size_t i = 0; i < count; ++i) {
        uint32_t raw = p[i].type1.data & 0xFFF;
        ring[ring_pos++] = (uint16_t)raw;
        if (ring_pos >= ringSize) ring_pos = 0;
      }

      size_t samples_needed = (size_t)BAR_COUNT * DECIMATE_PER_FRAME;
      ssize_t start_idx = (ssize_t)ring_pos - (ssize_t)samples_needed;
      if (start_idx < 0) start_idx += ringSize;

      for (size_t b = 0; b < BAR_COUNT; ++b) {
        uint32_t acc = 0;
        for (size_t d = 0; d < DECIMATE_PER_FRAME; ++d) {
          size_t idx = (start_idx + b * DECIMATE_PER_FRAME + d) % ringSize;
          acc += ring[idx];
        }
        uint16_t avg = (uint16_t)(acc / DECIMATE_PER_FRAME);
        uint32_t height = ((uint32_t)avg * BAR_MAX_HEIGHT_PIXELS) / 4095u;
        if (height > BAR_MAX_HEIGHT_PIXELS) height = BAR_MAX_HEIGHT_PIXELS;
        local_out[b] = (uint16_t)height;
      }

      // Copy local_out into shared_out (mutex-protected) and mark bufferReady.
      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // copy BAR_COUNT * 2 bytes
        memcpy((void*)shared_out, (void*)local_out, sizeof(uint16_t) * BAR_COUNT);
        bufferReady = true;
        xSemaphoreGive(bufferMutex);
        // try to offer the new frame (non-blocking copy)
if (!plotter.offerFrame(shared_out, BAR_COUNT)) {
    // offerFrame failed (mutex contention); it's okay — skip and let next ADC data try again
}
      } else {
        // If we can't get mutex quickly, just skip (main loop will consume later)
        // This avoids blocking ADC processing.
      }
    //vTaskDelay(pdMS_TO_TICKS(1));// 
    } else if (ret != ESP_OK) {
      Serial.print("adc_continuous_read error: ");
      Serial.println(ret);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  free(local_out);
  free(ring);
  vTaskDelete(NULL);
}

// ---------- setup & loop integrating tear-free timing ----------
void setup() {
  Serial.begin(115200);
  delay(50);

  tft.init();
  tft.initDMA();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(0x0000);
 // Initial display timing setup for even frames
  tft.startWrite();
  tft.writecommand(ST7789_FRCTR2);
  tft.writedata(0x0b);
  tft.writecommand(ST7789_PORCTRL);
  tft.writedata(0x16);
  tft.writedata(0x3f);
  tft.writedata(0x01); // enable separate porch control
  tft.writedata(0x33); // porch control in idle mode
  tft.writedata(0x33); // porch control in partial mode
  tft.endWrite();
  plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
  
// initialize tear-free timing inside BarPlotter:
// optional params: var1,var2,var3,var4,var5,var6, refreshRate
plotter.initTearFree(0x0b, 0x16, 0x3f, 0x1a, 0x40, 0x3f, 33.0f);

  bufferMutex = xSemaphoreCreateMutex();
  if (!bufferMutex) {
    Serial.println("buffer mutex create failed");
    while (1) delay(1000);
  }


 

  // setup ADC continuous DMA
  setup_adc_continuous();

  // create the reader task (pinned to core 1 as before)
  xTaskCreatePinnedToCore(adc_reader_task, "adc_reader", 8192, NULL, 2, NULL, 1);



  lastDisplayTime = millis();
}

void loop() {
  #ifdef TEARING_DEBUG
  serial_tft_helper();
  #endif

  
  // Non-blocking: small sleep to yield CPU
  delay(1);
}
