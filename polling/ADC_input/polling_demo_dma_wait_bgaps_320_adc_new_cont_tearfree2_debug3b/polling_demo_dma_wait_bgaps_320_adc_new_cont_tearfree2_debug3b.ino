// continuous_adc_plot_esp32.ino
// Uses ADC continuous (DMA) driver from ESP-IDF v5 (available in Arduino core 3.3.2)
// and feeds the BarPlotter as data source.

#include "BarPlotter.h"
#include <TFT_eSPI.h>

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "driver/adc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

#define SCREEN_UPDATE_RATE_MS 33     // ~30 FPS

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// ADC continuous settings
#define ADC_SAMPLE_FREQ_HZ    41000UL   // requested ADC sample frequency
#define ADC_READ_BUF_BYTES    (8 * 1024) // DMA internal pool size (bytes) - adjust if needed
#define ADC_FRAME_SIZE        320      // conversion frame size in bytes (conv_frame_size)
#define ADC_READ_TIMEOUT_MS   50        // read timeout

// decimation: how many ADC samples to skip to produce one output bar
// If your ADC run at 20kHz and BAR_COUNT is 320 for a single frame, you may need to tune this.
#define DECIMATE_PER_FRAME    2

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ADC handle
static adc_continuous_handle_t adc_handle = NULL;

// buffer for reading raw ADC bytes from driver
static uint8_t *adc_read_buffer = nullptr;
static volatile bool adc_running = false;

// Plot generation frequency (Hz) - default 50 Hz
static volatile float plot_rate_hz = 11.81f;

// Stats
static volatile uint32_t dropped_frames = 0;
static volatile uint32_t generated_frames = 0;

void setPlotRateHz(float hz) {
  if (hz < 1.0f) hz = 1.0f;
  if (hz > 120.0f) hz = 120.0f;
  plot_rate_hz = hz;
  plotter.setFrameRate(hz);
}

// -------- ADC and reader/plotter task --------
void setup_adc_continuous()
{
  // Configure ADC1 channel attenuation & width (oneshot config functions)
  // Use ADC1_CH0 -> GPIO36 (VP)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  // create continuous handle config
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

  // prepare pattern for one channel (ADC1_CH0)
  static adc_digi_pattern_config_t pattern[1];
  pattern[0].atten = ADC_ATTEN_DB_11;
  pattern[0].channel = ADC_CHANNEL_0; // ADC channel index relative to unit's enum (ADC_CHANNEL_0)
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = ADC_BITWIDTH_12; // 12-bit

  // configure continuous conversion behavior
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

  // allocate reading buffer
  adc_read_buffer = (uint8_t*)malloc(ADC_READ_BUF_BYTES);
  if (!adc_read_buffer) {
    Serial.println("adc_read_buffer malloc failed");
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  // start continuous acquisition
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

// Task: read ADC data from DMA and prepare a frame for the plotter.
// This task fills a circular buffer of the latest samples and periodically
// hands a derived uint16_t array to plotter.startBarPlot() at plot_rate_hz.
void adc_reader_task(void *pv)
{
  // We'll maintain a simple ring buffer of last N values (raw 12-bit)
  const size_t ringSize = BAR_COUNT * DECIMATE_PER_FRAME * 4; // more headroom
  uint16_t *ring = (uint16_t*)malloc(sizeof(uint16_t) * ringSize);
  if (!ring) {
    Serial.println("ring alloc failed");
    vTaskDelete(NULL);
    return;
  }
  size_t ring_pos = 0;

  // temporary output array for plotter
  uint16_t *out = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  if (!out) {
    Serial.println("out alloc failed");
    free(ring);
    vTaskDelete(NULL);
    return;
  }

  // timing variables
  TickType_t lastPlotTick = xTaskGetTickCount();
  uint32_t local_period_ms = (uint32_t)(1000.0f / plot_rate_hz + 0.5f);
  TickType_t period_ticks = pdMS_TO_TICKS(local_period_ms);

  while (1) {
    if (!adc_running) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Read any available ADC data (non-blocking-ish with timeout)
    uint32_t out_len = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, adc_read_buffer, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
      // no data this iteration - fall through to timing check
    } else if (ret == ESP_OK && out_len > 0) {
      // parse frames: out_len is bytes; each result has SOC_ADC_DIGI_RESULT_BYTES bytes.
      // transport to typed struct
      size_t item_size = sizeof(adc_digi_output_data_t);
      size_t count = out_len / item_size;
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)adc_read_buffer;

      for (size_t i = 0; i < count; ++i) {
        // TYPE1 format: use .type1.data, .type1.unit, .type1.channel
        uint32_t raw = p[i].type1.data & 0xFFF; // 12-bit data
        // store into ring buffer
        ring[ring_pos++] = (uint16_t)raw;
        if (ring_pos >= ringSize) ring_pos = 0;
      }
    } else if (ret != ESP_OK) {
      Serial.print("adc_continuous_read error: ");
      Serial.println(ret);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // recalc period ticks if plot_rate_hz changed
    {
      float hz = plot_rate_hz; // volatile read once
      uint32_t msec = (uint32_t)(1000.0f / hz + 0.5f);
      if (msec == 0) msec = 1;
      TickType_t new_period = pdMS_TO_TICKS(msec/2);
      if (new_period != period_ticks) {
        period_ticks = new_period;
      }
    }

    // Check whether it's time to prepare the next plotted frame.
    TickType_t now = xTaskGetTickCount();
    if ((now - lastPlotTick) >= period_ticks) {
      // advance lastPlotTick in multiples of period_ticks to avoid drift accumulation
      TickType_t delta = now - lastPlotTick;
      TickType_t steps = delta / period_ticks;
      lastPlotTick += steps * period_ticks;

      // produce a decimated snapshot to feed the plot - take last (ring) samples and decimate
      size_t samples_needed = (size_t)BAR_COUNT * DECIMATE_PER_FRAME;
      // start index of the block to decimate (most recent samples)
      ssize_t start_idx = (ssize_t)ring_pos - (ssize_t)samples_needed;
      if (start_idx < 0) start_idx += ringSize;

      for (size_t b = 0; b < BAR_COUNT; ++b) {
        // average DECIMATE_PER_FRAME samples for each bar
        uint32_t acc = 0;
        for (size_t d = 0; d < DECIMATE_PER_FRAME; ++d) {
          size_t idx = (start_idx + b * DECIMATE_PER_FRAME + d) % ringSize;
          acc += ring[idx];
        }
        uint16_t avg = (uint16_t)(acc / DECIMATE_PER_FRAME);
        // map 12-bit (0..4095) to plot height (0..BAR_MAX_HEIGHT_PIXELS)
        uint32_t height = ((uint32_t)avg * BAR_MAX_HEIGHT_PIXELS) / 4095u;
        if (height > BAR_MAX_HEIGHT_PIXELS) height = BAR_MAX_HEIGHT_PIXELS;
        out[b] = (uint16_t)height;
      }

      // Attempt to hand snapshot to plotter. startBarPlot will copy quickly into internal buffer.
      // startBarPlot will return immediately (or drop if it cannot take mutex).
      if (!plotter.startBarPlot(out, BAR_COUNT)) {
        dropped_frames++;
      } else {
        generated_frames++;
      }
    }

    // tiny yield to let other tasks run - keep this light so adc loop remains responsive
    taskYIELD();
  }

  // never reached
  free(out);
  free(ring);
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  tft.init();
  tft.initDMA();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(0x0000);

  plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
  setPlotRateHz(plot_rate_hz); // set default plot rate in plotter internals

  //setup ADC continuous DMA
  setup_adc_continuous();

  // create the reader task to transform ADC stream into plot frames
  xTaskCreatePinnedToCore(adc_reader_task, "adc_reader", 8192, NULL, 2, NULL, 1);
}

// Serial tuning state (similar to original TEARING_DEBUG)
uint8_t selected = 1; // 1..9
bool serial_debug_enabled = false;

void serial_plotter_helper() {
  if (Serial.available() == 0) return;
  char input = Serial.read();

  if (input >= '1' && input <= '9') {
    selected = input - '0';
  } else if (input == 'd' || input == 'D') {
    serial_debug_enabled = !serial_debug_enabled;
    plotter.enableDebug(serial_debug_enabled);
    Serial.printf("Debug %s\n", serial_debug_enabled ? "ENABLED" : "DISABLED");
    return;
  } else if (input == 's' || input == 'S') {
    plotter.printConfig();
    Serial.printf("Frames generated: %u, dropped: %u\n", (unsigned)generated_frames, (unsigned)dropped_frames);
    Serial.printf("Plotter state: %s\n", plotter.frameStateString());
    return;
  } else if (input == 'r' || input == 'R') {
    // toggle: print current target framerate
    Serial.printf("Target plot generation rate: %.2f Hz\n", plot_rate_hz);
    return;
  } else if (input == '-' || input == '_') {
    // decrement selected
    switch (selected) {
      case 1: { uint8_t v = plotter._even_frctr2; v--; plotter.setEvenTiming(v, plotter._even_fporch, plotter._even_bporch); break; }
      case 2: { uint8_t v = plotter._even_fporch; v--; plotter.setEvenTiming(plotter._even_frctr2, v, plotter._even_bporch); break; }
      case 3: { uint8_t v = plotter._even_bporch; v--; plotter.setEvenTiming(plotter._even_frctr2, plotter._even_fporch, v); break; }
      case 4: { uint8_t v = plotter._odd_frctr2; v--; plotter.setOddTiming(v, plotter._odd_fporch, plotter._odd_bporch); break; }
      case 5: { uint8_t v = plotter._odd_fporch; v--; plotter.setOddTiming(plotter._odd_frctr2, v, plotter._odd_bporch); break; }
      case 6: { uint8_t v = plotter._odd_bporch; v--; plotter.setOddTiming(plotter._odd_frctr2, plotter._odd_fporch, v); break; }
      case 7: { uint32_t p = plotter._phase_send_offset; p = (p > 100) ? p - 100 : 0; plotter.setPhaseOffsetUs(p); break; }
      case 8: { uint32_t m = plotter._mid_offset; m = (m > 50) ? m - 50 : 0; plotter.setMidOffsetUs(m); break; }
      case 9: { float r = plot_rate_hz; r = max(1.0f, r - 0.01f); setPlotRateHz(r); break; }
    }
    plotter.printConfig();
  } else if (input == '=' || input == '+') {
    // increment selected
    switch (selected) {
      case 1: { uint8_t v = plotter._even_frctr2; v++; plotter.setEvenTiming(v, plotter._even_fporch, plotter._even_bporch); break; }
      case 2: { uint8_t v = plotter._even_fporch; v++; plotter.setEvenTiming(plotter._even_frctr2, v, plotter._even_bporch); break; }
      case 3: { uint8_t v = plotter._even_bporch; v++; plotter.setEvenTiming(plotter._even_frctr2, plotter._even_fporch, v); break; }
      case 4: { uint8_t v = plotter._odd_frctr2; v++; plotter.setOddTiming(v, plotter._odd_fporch, plotter._odd_bporch); break; }
      case 5: { uint8_t v = plotter._odd_fporch; v++; plotter.setOddTiming(plotter._odd_frctr2, v, plotter._odd_bporch); break; }
      case 6: { uint8_t v = plotter._odd_bporch; v++; plotter.setOddTiming(plotter._odd_frctr2, plotter._odd_fporch, v); break; }
      case 7: { uint32_t p = plotter._phase_send_offset; p += 100; plotter.setPhaseOffsetUs(p); break; }
      case 8: { uint32_t m = plotter._mid_offset; m += 50; plotter.setMidOffsetUs(m); break; }
      case 9: { float r = plot_rate_hz; r = min(120.0f, r + 0.01f); setPlotRateHz(r); break; }
    }
    plotter.printConfig();
  }
}

void loop() {
  // main loop only prints benchmarks (the actual plotting is driven by the reader task)
  serial_plotter_helper();
  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    // print a small heartbeat + stats
    Serial.printf("Plot rate target: %.2f Hz | generated: %u | dropped: %u | plotState: %s\n", plot_rate_hz, (unsigned)generated_frames, (unsigned)dropped_frames, plotter.frameStateString());
  }
  delay(100);
}
