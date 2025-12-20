// continuous_adc_plot_esp32.ino
// ADC continuous (DMA) driver from ESP-IDF v5 + BarPlotter.
// Triggering affects buffer-copying to the plotter (not TFT frame rate).

#include <IRremote.h> // Arduino-IRremote (IrReceiver)
#include "BarPlotter.h"
#include <TFT_eSPI.h>

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "driver/adc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

#define IR_PIN 15      // change if needed
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
#define DECIMATE_PER_FRAME    2

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ADC handle
static adc_continuous_handle_t adc_handle = NULL;

// buffer for reading raw ADC bytes from driver
static uint8_t *adc_read_buffer = nullptr;
static volatile bool adc_running = false;

// Plot generation frequency (Hz) - TFT/frame sync ONLY. Do NOT let trigger change this.
static volatile float plot_rate_hz = 11.81f;

// Stats
static volatile uint32_t dropped_frames = 0;
static volatile uint32_t generated_frames = 0;

void setPlotRateHz(float hz) {
  // keep this function for manual tuning; trigger must NOT call this.
  if (hz < 1.0f) hz = 1.0f;
  if (hz > 120.0f) hz = 120.0f;
  plot_rate_hz = hz;
  plotter.setFrameRate(hz);
}

// ----- Triggering / copy-management state -----
static volatile bool trigger_enabled = false;
static volatile int trigger_slope = 1; // 1 = rising, -1 = falling
static volatile uint16_t trigger_value = 2048; // 0..4095 ADC raw
// Trigger hold: how long after last crossing we consider ourselves "triggered" (ms)
static volatile uint32_t trigger_hold_ms = 500;
static volatile uint32_t last_trigger_time_ms = 0;

// Copy frequency used while triggered (how often to attempt additional immediate copies, Hz)
static volatile float triggered_copy_freq_hz = 30.0f; // default copy frequency while triggered
// Minimum interval between triggered copies (derived)
static inline uint32_t triggered_copy_interval_ms() {
  float f = triggered_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}

// Optional fallback copy frequency to use when NOT triggered (this does NOT replace plot_rate_hz)
static volatile float fallback_copy_freq_hz = 5.0f; // unused for periodic sync, only used if you want to push less frequently in non-trigger mode via IR action
static inline uint32_t fallback_copy_interval_ms() {
  float f = fallback_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}

// When a trigger crossing is detected we will attempt an immediate snapshot copy.
// To avoid spamming the plotter we also ensure a minimum interval between these immediate copies:
static volatile uint32_t last_immediate_copy_ms = 0;
static volatile uint32_t minimum_immediate_copy_interval_ms = 20; // 50 Hz max immediate copy by default

// ---------- ADC and reader/plotter task --------
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

// Helper: produce a decimated snapshot into 'out' using the current ring buffer content.
// This is the same mapping as before: average DECIMATE_PER_FRAME samples per bar, map to pixel height.
static void produce_snapshot_from_ring(uint16_t *ring, size_t ringSize, size_t ring_pos, uint16_t *out) {
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
    out[b] = (uint16_t)height;
  }
}

// Task: read ADC data from DMA and prepare frames.
void adc_reader_task(void *pv)
{
  const size_t ringSize = BAR_COUNT * DECIMATE_PER_FRAME * 4; // headroom
  uint16_t *ring = (uint16_t*)malloc(sizeof(uint16_t) * ringSize);
  if (!ring) {
    Serial.println("ring alloc failed");
    vTaskDelete(NULL);
    return;
  }
  size_t ring_pos = 0;

  uint16_t *out = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  if (!out) {
    Serial.println("out alloc failed");
    free(ring);
    vTaskDelete(NULL);
    return;
  }

  // Periodic plotting tick uses plot_rate_hz (TFT sync) as before:
  TickType_t lastPlotTick = xTaskGetTickCount();
  uint32_t local_period_ms = (uint32_t)(1000.0f / plot_rate_hz + 0.5f);
  TickType_t period_ticks = pdMS_TO_TICKS(local_period_ms);

  // For trigger detection: keep previous sample
  uint16_t prev_sample = 0;

  // For triggered repeat-copy scheduling:
  uint32_t last_triggered_copy_ms_local = 0;

  while (1) {
    if (!adc_running) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Read ADC bytes (non-blocking-ish)
    uint32_t out_len = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, adc_read_buffer, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
      // no data this iteration
    } else if (ret == ESP_OK && out_len > 0) {
      size_t item_size = sizeof(adc_digi_output_data_t);
      size_t count = out_len / item_size;
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)adc_read_buffer;

      for (size_t i = 0; i < count; ++i) {
        uint32_t raw = p[i].type1.data & 0xFFF; // 12-bit
        ring[ring_pos++] = (uint16_t)raw;
        if (ring_pos >= ringSize) ring_pos = 0;

        // Trigger detection logic (sample-level)
        if (trigger_enabled) {
          uint16_t s = (uint16_t)raw;
          bool crossed = false;
          if (trigger_slope > 0) {
            if ((uint32_t)prev_sample < (uint32_t)trigger_value && (uint32_t)s >= (uint32_t)trigger_value) crossed = true;
          } else {
            if ((uint32_t)prev_sample > (uint32_t)trigger_value && (uint32_t)s <= (uint32_t)trigger_value) crossed = true;
          }
          if (crossed) {
            last_trigger_time_ms = millis();
            // Attempt an immediate snapshot copy (respecting minimum_immediate_copy_interval_ms)
            uint32_t nowms = millis();
            if ((nowms - last_immediate_copy_ms) >= minimum_immediate_copy_interval_ms) {
              // produce snapshot & attempt to push
              produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
              // try to hand to plotter immediately - startBarPlot returns false if it cannot accept
              if (!plotter.startBarPlot(out, BAR_COUNT)) {
                dropped_frames++;
                // optional: you could store one 'pending trigger snapshot' to attempt later,
                // but to keep logic simple we do a single immediate attempt per crossing.
              } else {
                generated_frames++;
              }
              last_immediate_copy_ms = nowms;
              last_triggered_copy_ms_local = nowms;
              Serial.printf("Trigger immediate copy (value=%u)\n", trigger_value);
            }
          }
          prev_sample = s;
        } else {
          prev_sample = (uint16_t)raw;
        }
      }
    } else if (ret != ESP_OK) {
      Serial.print("adc_continuous_read error: ");
      Serial.println(ret);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // recalc period_ticks if plot_rate_hz changed externally via serial (not trigger)
    {
      float hz = plot_rate_hz; // volatile read
      uint32_t msec = (uint32_t)(1000.0f / hz + 0.5f);
      if (msec == 0) msec = 1;
      TickType_t new_period = pdMS_TO_TICKS(msec);
      if (new_period != period_ticks) {
        period_ticks = new_period;
      }
    }

    // Check triggered-repeat copying: while inside trigger_hold_ms after last crossing,
    // attempt additional snapshot copies at triggered_copy_freq_hz (respecting minimum intervals).
    if (trigger_enabled && (millis() - last_trigger_time_ms) <= trigger_hold_ms) {
      uint32_t nowms = millis();
      uint32_t trig_interval = triggered_copy_interval_ms();
      if ((nowms - last_triggered_copy_ms_local) >= trig_interval) {
        // produce snapshot & attempt to push
        produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
        if (!plotter.startBarPlot(out, BAR_COUNT)) {
          dropped_frames++;
        } else {
          generated_frames++;
        }
        last_triggered_copy_ms_local = nowms;
      }
    }

    // Periodic plotting synchronized with TFT frame (unchanged behavior)
    TickType_t now = xTaskGetTickCount();
    if ((now - lastPlotTick) >= period_ticks) {
      TickType_t delta = now - lastPlotTick;
      TickType_t steps = delta / period_ticks;
      lastPlotTick += steps * period_ticks;

      // produce periodic decimated snapshot and hand to plotter (this is the normal TFT-synced path)
      produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
      if (!plotter.startBarPlot(out, BAR_COUNT)) {
        dropped_frames++;
      } else {
        generated_frames++;
      }
    }

    // yield briefly
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
  setPlotRateHz(plot_rate_hz); // set default plot rate in plotter internals (manual tuning possible)

  //setup ADC continuous DMA
  setup_adc_continuous();

  // create the reader task
  xTaskCreatePinnedToCore(adc_reader_task, "adc_reader", 8192, NULL, 2, NULL, 1);

  // IR setup
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.printf("IR receiver started on pin %d\n", IR_PIN);
}

// Serial tuning helpers (unchanged)
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
    Serial.printf("Target plot generation rate (TFT sync): %.2f Hz\n", plot_rate_hz);
    return;
  } else if (input == '-' || input == '_') {
    // decrement selected - keep same mapping as before
    switch (selected) {
      case 1: { uint8_t v = plotter._even_frctr2; v--; plotter.setEvenTiming(v, plotter._even_fporch, plotter._even_bporch); break; }
      case 2: { uint8_t v = plotter._even_fporch; v--; plotter.setEvenTiming(plotter._even_frctr2, v, plotter._even_bporch); break; }
      case 3: { uint8_t v = plotter._even_bporch; v--; plotter.setEvenTiming(plotter._even_frctr2, plotter._even_fporch, v); break; }
      case 4: { uint8_t v = plotter._odd_frctr2; v--; plotter.setOddTiming(v, plotter._odd_fporch, plotter._odd_bporch); break; }
      case 5: { uint8_t v = plotter._odd_fporch; v--; plotter.setOddTiming(plotter._odd_frctr2, v, plotter._odd_bporch); break; }
      case 6: { uint8_t v = plotter._odd_bporch; v--; plotter.setOddTiming(plotter._odd_frctr2, plotter._odd_fporch, v); break; }
      case 7: { uint32_t p = plotter._phase_send_offset; p = (p > 100) ? p - 100 : 0; plotter.setPhaseOffsetUs(p); break; }
      case 8: { uint32_t m = plotter._mid_offset; m = (m > 50) ? m - 50 : 0; plotter.setMidOffsetUs(m); break; }
      case 9: { float r = plot_rate_hz; r = max(1.0f, r - 0.01f); setPlotRateHz(r); break; } // manual TFT sync tuning
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
      case 9: { float r = plot_rate_hz; r = min(120.0f, r + 0.01f); setPlotRateHz(r); break; } // manual
    }
    plotter.printConfig();
  }
}

// Handle IR commands (called from loop())
void handle_ir() {
  if (!IrReceiver.decode()) return;

  // Only act on SAMSUNG protocol and address 0x07 (requested)
  if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x07) {
    uint32_t cmd = IrReceiver.decodedIRData.command;
    Serial.printf("IR (SAM+0x07) command: 0x%08X\n", (unsigned)cmd);

    // action nibble mapping (low 4 bits): same concept as before but act on copy-freq settings
    uint8_t action = cmd & 0x0F;
    uint8_t param = (cmd >> 4) & 0xFF;

    switch (action) {
      case 0xe: // toggle trigger enabled
        trigger_enabled = !trigger_enabled;
        Serial.printf("Trigger %s\n", trigger_enabled ? "ENABLED" : "DISABLED");
        if (!trigger_enabled) {
          // clear last_trigger_time so triggered-copy stops
          last_trigger_time_ms = 0;
        }
        break;
      case 0x1: // set slope = rising
        trigger_slope = 1;
        Serial.println("Trigger slope: RISING");
        break;
      case 0x2: // set slope = falling
        trigger_slope = -1;
        Serial.println("Trigger slope: FALLING");
        break;
      case 0x3: { // increase trigger_value
        uint16_t inc = (param == 0) ? 64 : (uint16_t)param * 16;
        uint32_t nv = (uint32_t)trigger_value + inc;
        if (nv > 4095) nv = 4095;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value increased -> %u\n", trigger_value);
        break;
      }
      case 0x4: { // decrease trigger_value
        uint16_t dec = (param == 0) ? 64 : (uint16_t)param * 16;
        uint32_t nv = (uint32_t)trigger_value;
        if (nv > dec) nv -= dec; else nv = 0;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value decreased -> %u\n", trigger_value);
        break;
      }
      case 0x5: { // set fallback copy frequency (Hz) - not the TFT frame rate
        float newf = (param == 0) ? 5.0f : (float)param;
        fallback_copy_freq_hz = newf;
        Serial.printf("Fallback copy freq set -> %.2f Hz (does not change TFT sync)\n", fallback_copy_freq_hz);
        break;
      }
      case 0x6: { // set triggered copy frequency (Hz)
        float newf = (param == 0) ? 30.0f : (float)param;
        if (newf < 1.0f) newf = 1.0f;
        triggered_copy_freq_hz = newf;
        Serial.printf("Triggered copy freq set -> %.2f Hz\n", triggered_copy_freq_hz);
        break;
      }
      case 0x7: // print status
        Serial.printf("STATUS: trigger=%s slope=%d value=%u hold_ms=%u triggered_copy=%.2f fallback_copy=%.2f\n",
                      trigger_enabled ? "ON" : "OFF", trigger_slope, trigger_value, (unsigned)trigger_hold_ms, triggered_copy_freq_hz, fallback_copy_freq_hz);
        break;
      default:
        Serial.printf("Unmapped action 0x%X (cmd 0x%08X)\n", action, (unsigned)cmd);
        break;
    }
  } else {
    // helpful feedback for learning remote codes
    //Serial.printf("IR ignored: proto=%u addr=0x%02X cmd=0x%08X\n",
    //              IrReceiver.decodedIRData.protocol,
    //              (unsigned)IrReceiver.decodedIRData.address,
    //              (unsigned)IrReceiver.decodedIRData.command);
  }

  IrReceiver.resume(); // receive next
}

void loop() {
  serial_plotter_helper();
  handle_ir();

  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.printf("Plot (TFT) rate: %.2f Hz | generated: %u | dropped: %u | trigger=%s | val=%u\n",
                  plot_rate_hz, (unsigned)generated_frames, (unsigned)dropped_frames,
                  trigger_enabled ? "ON" : "OFF", trigger_value);
  }
  //delay(100);
}
