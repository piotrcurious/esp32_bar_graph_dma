// continuous_adc_plot_esp32.ino
// Improved trigger + plotter coordination version
// ADC continuous (DMA) driver from ESP-IDF v5 + BarPlotter.
// IRremote-based control using provided Samsung key table.

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
//#define SCREEN_UPDATE_RATE_MS 33     // ~30 FPS  (not used directly)


// ---------- USER CONFIG ----------
#define TFT_ROTATION          1
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// ADC continuous settings
#define ADC_SAMPLE_FREQ_HZ    31000UL   // requested ADC sample frequency
#define ADC_READ_BUF_BYTES    (8 * 1024) // DMA internal pool size (bytes) - adjust if needed
#define ADC_FRAME_SIZE        640      // conversion frame size in bytes (conv_frame_size)
#define ADC_READ_TIMEOUT_MS   50        // read timeout (ms)

// decimation: how many ADC samples to skip to produce one output bar
#define DECIMATE_PER_FRAME    4

// ---------- IR KEY TABLE (user-supplied) ----------
enum KeyCode {
  KEY_POWER = 0x02,
  KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
  KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
  KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
  KEY_OK = 0x68, KEY_MENU = 0x79, KEY_RED = 0x6c, KEY_GREEN = 0x14,
  KEY_YELLOW = 0x15, KEY_BLUE = 0x16, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
  KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10, KEY_REWIND = 0x45, KEY_PLAY = 0x47,
  KEY_PAUSE = 0x4A, KEY_FORWARD = 0x48, KEY_STOP = 0x46, KEY_SETTINGS = 0x1A,
  KEY_INFO = 0x1F, KEY_SUBTITLES = 0x25, KEY_MUTE = 0x0F, KEY_NETFLIX = 0xF3,
  KEY_PRIME_VIDEO = 0xF4, KEY_GUIDE = 0x4F, KEY_SOURCE = 0x01
};

// ---------- FrameState constants (match BarPlotter's FrameState enum) ----------
#define FRAME_IDLE     0
#define FRAME_QUEUED   1
#define FRAME_CONSUMING 2
#define FRAME_SENT     3

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ADC handle
static adc_continuous_handle_t adc_handle = NULL;

// buffer for reading raw ADC bytes from driver
static uint8_t *adc_read_buffer = nullptr;
static volatile bool adc_running = false;

// Plot generation frequency (Hz) - TFT/frame sync ONLY. Do NOT let trigger change this.
//static volatile float plot_rate_hz = 11.81f;
static volatile float plot_rate_hz = 23.62f;
// Stats
static volatile uint32_t dropped_frames = 0;
static volatile uint32_t generated_frames = 0;

// ---------- Triggering / copy-management state -----
static volatile bool trigger_enabled = false;
static volatile int trigger_slope = 1; // 1 = rising, -1 = falling
static volatile uint16_t trigger_value = 2048; // 0..4095 ADC raw
// Trigger hold: how long after last crossing we consider ourselves "triggered" (ms)
static volatile uint32_t trigger_hold_ms = 500;
static volatile uint32_t last_trigger_time_ms = 0;

// Copy frequency used while triggered (how often to attempt additional immediate copies, Hz)
static volatile float triggered_copy_freq_hz = 30.0f; // default copy frequency while triggered
static inline uint32_t triggered_copy_interval_ms() {
  float f = triggered_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}

// Fallback copy frequency to use when trigger looks unreliable
static volatile float fallback_copy_freq_hz = 5.0f;
static inline uint32_t fallback_copy_interval_ms() {
  float f = fallback_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}

// When no crossing detected for > 2 * (1/fallback_copy_freq_hz) seconds, consider trigger unreliable:
static inline uint32_t fallback_unreliable_threshold_ms() {
  float f = fallback_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  // 2 * period = 2 * (1000 / f)
  return (uint32_t)(2.0f * (1000.0f / f) + 0.5f);
}

// To avoid spamming the plotter we enforce a minimum immediate copy interval:
static volatile uint32_t last_immediate_copy_ms = 0;
static volatile uint32_t minimum_immediate_copy_interval_ms = 40; // 50 Hz max immediate copy by default

// Single pending snapshot buffer (one-slot) - if an immediate/fallback copy fails we save one snapshot to retry later.
static uint16_t *pending_snapshot = nullptr;
static volatile bool pending_snapshot_valid = false;

// ---------- Trigger tuning additions ----------
static volatile uint16_t trigger_hysteresis = 8;        // LSBs around trigger_value to reduce noise spikes
static volatile uint32_t min_trigger_separation_ms = 50; // don't accept >1 crossing inside this window

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

// If a pending snapshot exists, attempt to push it and clear on success.
static void try_flush_pending() {
  if (pending_snapshot_valid && pending_snapshot) {
    if (plotter.startBarPlot(pending_snapshot, BAR_COUNT)) {
      generated_frames++;
      pending_snapshot_valid = false;
    } else {
      // still busy; keep pending, but count as dropped attempt for monitoring
      dropped_frames++;
    }
  }
}

// New: attempt_queue_snapshot - polite, frame-aware push (returns true if accepted or saved)
static bool attempt_queue_snapshot(uint16_t *snapshot) {
  // give priority to older pending
  try_flush_pending();

  int fs = plotter.getFrameState();
  bool queueEmpty = plotter.isQueueEmpty();

  // If plotter is consuming a frame, prefer to stash to pending (if empty)
  if (fs == FRAME_CONSUMING) {
    if (!pending_snapshot_valid && pending_snapshot) {
      memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
      pending_snapshot_valid = true;
      return true; // saved to pending
    } else {
      dropped_frames++;
      return false;
    }
  }

  // If idle or recently sent a frame (higher chance to accept), attempt to queue
  if ((fs == FRAME_IDLE) || (fs == FRAME_SENT) || queueEmpty) {
    if (plotter.startBarPlot(snapshot, BAR_COUNT)) {
      // accepted by plotter
      generated_frames++;
      return true;
    } else {
      // failed - try to save pending
      if (!pending_snapshot_valid && pending_snapshot) {
        memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
        pending_snapshot_valid = true;
        return true;
      } else {
        dropped_frames++;
        return false;
      }
    }
  }

  // fallback: try to save pending if empty
  if (!pending_snapshot_valid && pending_snapshot) {
    memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
    pending_snapshot_valid = true;
    return true;
  }

  // nothing possible
  dropped_frames++;
  return false;
}

// ---------- ADC reader task --------
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

  // allocate pending snapshot buffer
  pending_snapshot = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  pending_snapshot_valid = false;

  // Periodic plotting tick uses plot_rate_hz (TFT sync) as before:
  TickType_t lastPlotTick = xTaskGetTickCount();
  uint32_t local_period_ms = (uint32_t)(1000.0f / plot_rate_hz + 0.5f);
  TickType_t period_ticks = pdMS_TO_TICKS(local_period_ms);

  // For trigger detection: keep previous sample
  uint16_t prev_sample = 0;

  // track enable state locally to prime prev_sample on enable transition
  bool last_trigger_enabled_local = false;

  // guard rapid-fire trigger events
  uint32_t last_trigger_event_ms = 0;

  // For triggered repeat-copy scheduling:
  uint32_t last_triggered_copy_ms_local = 0;
  // For fallback repeat-copy scheduling:
  uint32_t last_fallback_copy_ms_local = 0;

  while (1) {
    if (!adc_running) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // If we have pending snapshot, try to flush it first (gives priority)
    try_flush_pending();

    // Detect trigger enable/disable transitions and prime prev_sample
    if (trigger_enabled != last_trigger_enabled_local) {
      last_trigger_enabled_local = trigger_enabled;
      if (last_trigger_enabled_local) {
        // On enable: take the most recent sample already in ring as prev_sample (avoid false immediate crossing)
        size_t last_idx = (ring_pos == 0) ? (ringSize - 1) : (ring_pos - 1);
        prev_sample = ring[last_idx];
        // avoid immediate fallback snaps until some time passes or we see a real crossing
        last_trigger_time_ms = millis();
        last_immediate_copy_ms = millis();
        // clear stale pending
        pending_snapshot_valid = false;
        last_trigger_event_ms = 0;
        if (false) Serial.println("Trigger ENABLED: primed prev_sample and timers.");
      } else {
        if (false) Serial.println("Trigger DISABLED");
      }
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

        // Trigger detection logic (sample-level) with hysteresis + min separation
        if (trigger_enabled) {
          uint16_t s = (uint16_t)raw;
          bool crossed = false;

          // hysteresis bounds
          uint32_t low_th = (trigger_value > trigger_hysteresis) ? (uint32_t)trigger_value - trigger_hysteresis : 0u;
          uint32_t high_th = (uint32_t)trigger_value + trigger_hysteresis;
          if (high_th > 4095u) high_th = 4095u;

          if (trigger_slope > 0) {
            // rising: require prev < low_th and now >= high_th
            if ((uint32_t)prev_sample < low_th && (uint32_t)s >= high_th) crossed = true;
          } else {
            // falling: require prev > high_th and now <= low_th
            if ((uint32_t)prev_sample > high_th && (uint32_t)s <= low_th) crossed = true;
          }

          if (crossed) {
            uint32_t nowms = millis();
            // enforce minimum separation between accepted trigger events
            if ((nowms - last_trigger_event_ms) >= min_trigger_separation_ms) {
              last_trigger_time_ms = nowms;
              last_trigger_event_ms = nowms;

              // Attempt an immediate snapshot copy (respecting minimum_immediate_copy_interval_ms)
              if ((nowms - last_immediate_copy_ms) >= minimum_immediate_copy_interval_ms) {
                produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
                attempt_queue_snapshot(out);
                last_immediate_copy_ms = nowms;
                last_triggered_copy_ms_local = nowms;
                //Serial.printf("Trigger immediate copy (value=%u)\n", trigger_value);
              }
            } else {
              // suppressed by min separation (debug optionally)
              // Serial.printf("Trigger suppressed (too-quick): now=%u last_event=%u\n", nowms, last_trigger_event_ms);
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
        produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
        attempt_queue_snapshot(out);
        last_triggered_copy_ms_local = nowms;
      }
    }

    // Fallback-unreliable mode: if trigger enabled but no crossing for > threshold,
    // produce snapshots at fallback_copy_freq_hz until a crossing occurs.
    if (trigger_enabled) {
      uint32_t nowms = millis();
      uint32_t threshold_ms = fallback_unreliable_threshold_ms();
      if ((nowms - last_trigger_time_ms) > threshold_ms) {
        // we're in fallback-unreliable mode
        uint32_t fb_interval = fallback_copy_interval_ms();
        if ((nowms - last_fallback_copy_ms_local) >= fb_interval) {
          // respect minimum_immediate_copy_interval_ms too
          if ((nowms - last_immediate_copy_ms) >= minimum_immediate_copy_interval_ms) {
            produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
            attempt_queue_snapshot(out);
            last_immediate_copy_ms = nowms;
            last_fallback_copy_ms_local = nowms;
            Serial.printf("Fallback copy (no crossing for %u ms, threshold %u ms)\n",
                          (unsigned)(nowms - last_trigger_time_ms), (unsigned)threshold_ms);
          } else {
            last_fallback_copy_ms_local = nowms;
          }
        }
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
      attempt_queue_snapshot(out);
    }

    // give short yield
    taskYIELD();
  }

  // never reached
  free(out);
  free(ring);
  if (pending_snapshot) free(pending_snapshot);
  vTaskDelete(NULL);
}

// -------------------- main/setup/loop --------------------
void setPlotRateHz(float hz) {
  // keep this function for manual tuning; trigger must NOT call this.
  if (hz < 1.0f) hz = 1.0f;
  if (hz > 120.0f) hz = 120.0f;
  plot_rate_hz = hz;
  plotter.setFrameRate(hz);
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

// Serial tuning helpers (unchanged semantics)
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
    uint32_t cmd_raw = IrReceiver.decodedIRData.command;
    uint8_t key = (uint8_t)(cmd_raw & 0xFF); // low byte is keycode for Samsung remotes
    Serial.printf("IR (SAM+0x07) command: 0x%02X (raw 0x%08X)\n", key, (unsigned)cmd_raw);

    switch (key) {
      case KEY_POWER:
        trigger_enabled = !trigger_enabled;
        Serial.printf("Trigger %s\n", trigger_enabled ? "ENABLED" : "DISABLED");
        if (!trigger_enabled) {
          last_trigger_time_ms = 0;
        } else {
          // prime timing to avoid immediate fallback and give ADC task a clean prev_sample
          last_immediate_copy_ms = millis();
          last_trigger_time_ms = millis();  // treat enable moment as "recent trigger"
        }
        break;

      case KEY_UP:
        trigger_slope = 1;
        Serial.println("Trigger slope: RISING");
        break;
      case KEY_DOWN:
        trigger_slope = -1;
        Serial.println("Trigger slope: FALLING");
        break;

      case KEY_LEFT: { // decrease trigger_value (coarse)
        uint16_t dec = 64;
        uint32_t nv = (uint32_t)trigger_value;
        if (nv > dec) nv -= dec; else nv = 0;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value decreased -> %u\n", trigger_value);
        break;
      }
      case KEY_RIGHT: { // increase trigger_value (coarse)
        uint16_t inc = 64;
        uint32_t nv = (uint32_t)trigger_value + inc;
        if (nv > 4095) nv = 4095;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value increased -> %u\n", trigger_value);
        break;
      }

      case KEY_VOL_UP: { // increase triggered copy freq
        triggered_copy_freq_hz = min(200.0f, triggered_copy_freq_hz + 5.0f);
        Serial.printf("Triggered copy freq -> %.2f Hz\n", triggered_copy_freq_hz);
        break;
      }
      case KEY_VOL_DOWN: { // decrease triggered copy freq
        triggered_copy_freq_hz = max(1.0f, triggered_copy_freq_hz - 5.0f);
        Serial.printf("Triggered copy freq -> %.2f Hz\n", triggered_copy_freq_hz);
        break;
      }

      case KEY_CH_UP: { // increase fallback copy freq
        fallback_copy_freq_hz = min(60.0f, fallback_copy_freq_hz + 1.0f);
        Serial.printf("Fallback copy freq -> %.2f Hz (thresh %u ms)\n", fallback_copy_freq_hz, (unsigned)fallback_unreliable_threshold_ms());
        break;
      }
      case KEY_CH_DOWN: { // decrease fallback copy freq
        fallback_copy_freq_hz = max(0.1f, fallback_copy_freq_hz - 1.0f);
        Serial.printf("Fallback copy freq -> %.2f Hz (thresh %u ms)\n", fallback_copy_freq_hz, (unsigned)fallback_unreliable_threshold_ms());
        break;
      }

      case KEY_OK:
        Serial.printf("STATUS: trigger=%s slope=%d value=%u hold_ms=%u triggered_copy=%.2f fallback_copy=%.2f pending=%d\n",
                      trigger_enabled ? "ON" : "OFF", trigger_slope, trigger_value, (unsigned)trigger_hold_ms, triggered_copy_freq_hz, fallback_copy_freq_hz, pending_snapshot_valid ? 1 : 0);
        break;

      case KEY_MENU: // toggle pending-retry behavior: clearing the pending slot
        if (pending_snapshot_valid) {
          pending_snapshot_valid = false;
          Serial.println("Pending snapshot CLEARED");
        } else {
          Serial.println("Pending snapshot slot EMPTY");
        }
        break;

      // numeric quick presets: set trigger_value proportionally (0..9)
      case KEY_0: case KEY_1: case KEY_2: case KEY_3: case KEY_4:
      case KEY_5: case KEY_6: case KEY_7: case KEY_8: case KEY_9: {
        uint8_t digit = 0;
        if (key == KEY_0) digit = 0;
        else if (key == KEY_1) digit = 1;
        else if (key == KEY_2) digit = 2;
        else if (key == KEY_3) digit = 3;
        else if (key == KEY_4) digit = 4;
        else if (key == KEY_5) digit = 5;
        else if (key == KEY_6) digit = 6;
        else if (key == KEY_7) digit = 7;
        else if (key == KEY_8) digit = 8;
        else if (key == KEY_9) digit = 9;
        // scale 0..9 -> 0..4095
        trigger_value = (uint16_t)(((uint32_t)digit * 4095) / 9);
        Serial.printf("Trigger quick-preset -> digit=%u value=%u\n", digit, trigger_value);
        break;
      }

      default:
        Serial.printf("Unmapped key 0x%02X\n", key);
        break;
    }
  } else {
    // ignored other protocols
  }

  IrReceiver.resume(); // receive next
}

void loop() {
  serial_plotter_helper();
  handle_ir();

  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.printf("Plot (TFT) rate: %.2f Hz | generated: %u | dropped: %u | trigger=%s | val=%u | fb_thresh_ms=%u | pending=%d\n",
                  plot_rate_hz, (unsigned)generated_frames, (unsigned)dropped_frames,
                  trigger_enabled ? "ON" : "OFF", trigger_value, (unsigned)fallback_unreliable_threshold_ms(), pending_snapshot_valid ? 1 : 0);
  }
  yield(); // let FreeRTOS schedule ADC/plot tasks
}
