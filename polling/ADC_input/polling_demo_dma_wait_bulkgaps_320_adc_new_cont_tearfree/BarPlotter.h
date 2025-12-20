#ifndef BAR_PLOTTER_H
#define BAR_PLOTTER_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "freertos/semphr.h" // ensure SemaphoreHandle_t is visible

class BarPlotter {
public:
    BarPlotter(TFT_eSPI& tft);
    ~BarPlotter();

    // Initialize tear-free timing subsystem (call after begin())
    bool initTearFree(uint8_t var1 = 0x0b, uint8_t var2 = 0x16, uint8_t var3 = 0x3f,
                      uint8_t var4 = 0x1a, uint8_t var5 = 0x40, uint8_t var6 = 0x3f,
                      float refreshRate = 33.0f);

    void stopTearFree();
    bool offerFrame(const uint16_t* data, size_t count);

    bool begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing);
    void startBarPlot(const uint16_t* dataArray, size_t count);
    bool isPlotting() const;
    bool isQueueEmpty() const;
    void endWrite();

    // public visual/state members (kept small)
    // (none required here; keep internals private)

private:
    // Tasks (static member functions)
    static void plotTask(void* pvParameters);
    static void timingTask(void* pvParameters); // <- added: timing task is a class static member
    // tear-free / timing internal state (private)
    SemaphoreHandle_t _tftMutex;        // protects startWrite()/pushImageDMA() register windows
    SemaphoreHandle_t _frameMutex;      // protects internal frame buffer and ready flag
    TaskHandle_t _timingTaskHandle;     // internal timing task
    uint16_t* _frameBuf;                // DMA-capable internal frame buffer
    volatile bool _frameReady;          // true when _frameBuf contains a new frame to consume

    // timing parameters
    uint8_t _tf_var1;
    uint8_t _tf_var2;
    uint8_t _tf_var3;
    uint8_t _tf_var4;
    uint8_t _tf_var5;
    uint8_t _tf_var6;
    float   _tf_refreshRate;

    // TFT reference and geometry
    TFT_eSPI& _tft;

    int _barCount;
    int _barWidth;
    int _barMaxHeight;
    int _plotX;
    int _plotY;
    int _plotSpacing;

    // Plot state
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
};

#endif // BAR_PLOTTER_H
