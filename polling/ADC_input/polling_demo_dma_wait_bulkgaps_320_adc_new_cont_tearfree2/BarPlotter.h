#ifndef BAR_PLOTTER_H
#define BAR_PLOTTER_H

#include <Arduino.h>
#include <TFT_eSPI.h>

class BarPlotter {
public:
    BarPlotter(TFT_eSPI& tft);
    ~BarPlotter();

    bool begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing);
    void startBarPlot(const uint16_t* dataArray, size_t count);
    bool isPlotting() const;
    bool isQueueEmpty() const;
    void endWrite();
    // setter helpers
void setFrameRate(float hz);      // update frame_us, mid_offset default etc.
void setPhaseOffsetUs(uint32_t us);
void setPorchStep(uint8_t step);

private:
    static void plotTask(void* pvParameters);

    TFT_eSPI& _tft;

    int _barCount;
    int _barWidth;
    int _barMaxHeight;
    int _plotX;
    int _plotY;
    int _plotSpacing;

    volatile bool _plotInProgress;
    volatile bool _plotQueueInProgress;

    const uint16_t* _plotDataArray;
    size_t _plotDataCount;
    size_t _currentBarIndex;
    TaskHandle_t _plotTaskHandle;

    uint16_t* _barSourceCopies;
    size_t _barSourcePixelsSingle;
    size_t _barSourceCopiesCount;
    size_t _barSourceRowsSingle;
    size_t _rowWidth;

    uint16_t* _dmaBarBuffer;
    size_t _dmaBarBufferPixels;

    // Pattern definition
    static const uint8_t _patternW = 4;
    static const uint8_t _patternH = 16;
    static const uint8_t _pattern[_patternH][_patternW];

    // Colors
    uint16_t _colorFill;
    uint16_t _colorExtra;
    uint16_t _colorFg;
    uint16_t _screenBg;
    uint16_t _gridColor;

    bool createTallBarSourceCopiesWithPattern();
    bool createPerBarDmaBuffer();
    void prepareDmaBarBufferForValue(uint16_t v);
    uint16_t* windowPtrForValueAndCopy(uint16_t v, size_t copyIndex);
    void clearSpacingAreas();

    // --- timing / PLL style nudge fields (single-buffer scheduler)
uint64_t _frame_us;           // frame interval in microseconds (1/refresh)
uint32_t _phase_send_offset;  // how many us before expected frame start to send "start-of-frame" timing write
uint32_t _mid_offset;         // offset (us) from push time to send alternate timing write (default frame_us/2)
uint8_t  _porch_step;         // tuning step for porch adjustments (small)
float    _target_rate_hz;     // for convenience


};

#endif // BAR_PLOTTER_H
