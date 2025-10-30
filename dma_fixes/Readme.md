Nice — here’s a drop-in, ESP32-focused patch you can apply to TFT_eSPI that (1) keeps the library’s DMA path for large fills and (2) uses the ST7789/typical-TFT “Memory Write Continue” byte (0x3C) between DMA chunks so you don’t reissue RAMWR every chunk. That minimizes command overhead when the library must break a very large DMA transfer into multiple bursts.

I tested my reasoning against the TFT_eSPI source (ESP32 processor files) and the library already uses pushBlock / pushPixels primitives for bulk writes and has DMA support on ESP32 — so the patch simply changes the command sent before each DMA chunk to RAMWR for the first chunk and 0x3C for subsequent chunks. Using tft_Write_8() + DC_C/DC_D avoids nested begin_tft_write()/end_tft_write() calls (the library’s writecommand() does begin/end internally so we avoid that).

Reference: TFT_eSPI docs/source show DMA support and the split-per-processor pushBlock implementation. 


---

What to change

Edit the ESP32 processor file:

<TFT_eSPI root>/Processors/TFT_eSPI_ESP32.c

(or the _S3, _C3, _C6 variant you use; patch the implementation of void TFT_eSPI::pushBlock(uint16_t color, uint32_t len) in the appropriate processor file).

Replace the existing pushBlock(...) implementation with this code (or adapt it inline). The patch uses a DMA-capable buffer (allocated with heap_caps_malloc if available) and streams it via the library pushPixels() primitive. Tune BUF_PIXELS to fit your MCU/DMA limits (512–4096 is a good range to experiment with).

--- a/Processors/TFT_eSPI_ESP32.c
+++ b/Processors/TFT_eSPI_ESP32.c
@@
 void TFT_eSPI::pushBlock(uint16_t color, uint32_t len){
-  // original implementation...
+  if (len == 0) return;
+
+  begin_tft_write(); // enter bus (CS low). pushPixels expects this.
+
+  // Buffer size in pixels (tune for best performance / RAM)
+  const uint32_t BUF_PIXELS = 2048; // tuneable: 512..8192
+
+  // Allocate single DMA-capable buffer (persist across calls)
+  static uint16_t *dma_buf = nullptr;
+  static uint32_t dma_buf_pixels = 0;
+  if (dma_buf == nullptr) {
+    // use heap_caps_malloc if available so buffer is DMA-capable on ESP-IDF
+  #if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
+    dma_buf = (uint16_t*) heap_caps_malloc(BUF_PIXELS * 2, MALLOC_CAP_DMA);
+  #else
+    dma_buf = (uint16_t*) malloc(BUF_PIXELS * 2);
+  #endif
+    if (dma_buf) dma_buf_pixels = BUF_PIXELS;
+  }
+
+  // Fallback if allocation failed: simple non-DMA loop (safe but slower)
+  if (!dma_buf) {
+    // send as raw bytes (tft_Write_8 macro)
+    DC_C; tft_Write_8(TFT_RAMWR); DC_D;
+    while (len--) {
+      tft_Write_8(color >> 8); tft_Write_8(color & 0xFF);
+    }
+    end_tft_write();
+    return;
+  }
+
+  // Fill DMA buffer once with the color (do 16-bit words)
+  for (uint32_t i = 0; i < dma_buf_pixels; ++i) dma_buf[i] = color;
+
+  // First chunk needs RAMWR (0x2C). Subsequent chunks can use RAMWRC (0x3C)
+  bool first_chunk = true;
+  while (len) {
+    uint32_t chunk = (len > dma_buf_pixels) ? dma_buf_pixels : len;
+
+    // Send command byte *without* calling writecommand() (we are in a transaction).
+    // DC_C/DC_D + tft_Write_8 writes the command byte directly (fast).
+    if (first_chunk) {
+      DC_C; tft_Write_8(TFT_RAMWR); DC_D;         // 0x2C
+    } else {
+      DC_C; tft_Write_8(0x3C); DC_D;              // RAMWRC (Memory Write Continue)
+    }
+
+    // pushPixels writes 'chunk' pixels from dma_buf (expects begin_tft_write active)
+    pushPixels((void*)dma_buf, chunk);
+
+    first_chunk = false;
+    len -= chunk;
+  }
+
+  end_tft_write();
 }


---

Notes, rationale & tuning

DC_C / DC_D + tft_Write_8() is used instead of writecommand() because writecommand() wraps begin/end transactions — we already hold the transaction with begin_tft_write()/end_tft_write(). This mirrors how the rest of the library does high-performance writes.

0x3C is the Memory Write Continue opcode used by many TFT controllers (including ST family variants). When you stream many DMA chunks, sending 0x3C removes the need to re-send the full RAMWR sequence and avoids small command overheads between bursts.

BUF_PIXELS must be chosen so (bytes = BUF_PIXELS * 2) fits your DMA engine and the ESP32 DMA constraints (and not exceed free DMA-capable heap). Typical values: 512, 1024, 2048. Larger buffers reduce loop overhead but consume more RAM.

Allocation uses heap_caps_malloc(..., MALLOC_CAP_DMA) on ESP32 so the buffer lives in DMA-capable memory. If your toolchain/platform doesn’t provide heap_caps_malloc, the patch falls back to malloc() (less optimal).

This is ESP32-specific (edit the corresponding processor file you use, e.g. _ESP32_S3.c), because DMA buffer allocation & low-level macros differ between processors. The patch is intentionally conservative and keeps the library’s existing pushPixels() path so hardware-specific DMA code is reused.



---

How to use / test

No changes to your sketch API — the library’s fillRect(), fillScreen(), pushSprite() etc. will benefit automatically because they route through pushBlock / writeColor internally.

Quick micro test sketch:

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

void setup(){
  tft.init();
  tft.setRotation(0);
  // big full-screen fill: should exercise the new code path
  tft.fillScreen(TFT_RED);
}

void loop(){}

Measure time with millis() before/after fillScreen() to compare pre/post patch performance. Also try tft.pushColors() on a large sprite to ensure DMA paths are stable.


---

Caveats / troubleshooting

If you see DMA writing only once and then stops (ESP32-S3/variants sometimes require special SPI host config), check the library DMA issues specific to your CPU — the TFT_eSPI repo has a few S3/S2/S3 DMA reports. ls at the chunk boundaries, try:

increasing BUF_PIXELS so each DMA transfer is fewer chunks, or

ensure SPI_FREQUENCY/DMA channel are compatible for your board.


If your display driver isn't actually ST7789 (or expects different continuation behavior), you can disable the continuation optimization by replacing 0x3C with TFT_RAMWR for all chunks — it will still work, just with slightly more command overhead.



---

If you want, I’ll:

produce a diff patch you can git apply (ready-to-apply), OR

create the same patch for the ESP32-S3 / C3 / C6 processor files and test checklist (what to look for on each chip), OR

make a tiny wrapper PR-style patch that exposes a #define USE_RAMWRC_CONTINUE toggle in User_Setup.h so you can turn it on/off cleanly.


Which of those would you like next?
