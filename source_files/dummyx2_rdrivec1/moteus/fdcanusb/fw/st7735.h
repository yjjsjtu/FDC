// Copyright 2025 fdcanusb contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <cstdio>
#include "mbed.h"
#include "fw/millisecond_timer.h"
#include "fw/font_arial.h"
#include "fw/stm32_dma.h"

namespace fw {

// Minimal 5x8 font: ASCII 0x20-0x5A
namespace font5x8 {
static const uint8_t DATA[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // space
  {0x00,0x00,0x5F,0x00,0x00}, // !
  {0x00,0x07,0x00,0x07,0x00}, // "
  {0x14,0x7F,0x14,0x7F,0x14}, // #
  {0x24,0x2A,0x7F,0x2A,0x12}, // $
  {0x23,0x13,0x08,0x64,0x62}, // %
  {0x36,0x49,0x55,0x22,0x50}, // &
  {0x00,0x05,0x03,0x00,0x00}, // '
  {0x00,0x1C,0x22,0x41,0x00}, // (
  {0x00,0x41,0x22,0x1C,0x00}, // )
  {0x08,0x2A,0x1C,0x2A,0x08}, // *
  {0x08,0x08,0x3E,0x08,0x08}, // +
  {0x00,0x50,0x30,0x00,0x00}, // ,
  {0x08,0x08,0x08,0x08,0x08}, // -
  {0x00,0x60,0x60,0x00,0x00}, // .
  {0x20,0x10,0x08,0x04,0x02}, // /
  {0x3E,0x51,0x49,0x45,0x3E}, // 0
  {0x00,0x42,0x7F,0x40,0x00}, // 1
  {0x42,0x61,0x51,0x49,0x46}, // 2
  {0x21,0x41,0x45,0x4B,0x31}, // 3
  {0x18,0x14,0x12,0x7F,0x10}, // 4
  {0x27,0x45,0x45,0x45,0x39}, // 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 6
  {0x01,0x71,0x09,0x05,0x03}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x06,0x49,0x49,0x29,0x1E}, // 9
  {0x00,0x36,0x36,0x00,0x00}, // :
  {0x00,0x56,0x36,0x00,0x00}, // ;
  {0x00,0x08,0x14,0x22,0x41}, // <
  {0x14,0x14,0x14,0x14,0x14}, // =
  {0x41,0x22,0x14,0x08,0x00}, // >
  {0x02,0x01,0x51,0x09,0x06}, // ?
  {0x32,0x49,0x79,0x41,0x3E}, // @
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x01,0x01}, // F
  {0x3E,0x41,0x41,0x51,0x32}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x04,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x7F,0x20,0x18,0x20,0x7F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x03,0x04,0x78,0x04,0x03}, // Y
  {0x61,0x51,0x49,0x45,0x43}, // Z
};
} // namespace font5x8

class ST7735 {
 public:
  struct Options {
    SPI_TypeDef* spi;
    DMA_Channel_TypeDef* tx_dma;
    GPIO_TypeDef* sck_port;
    uint16_t sck_pin;
    uint8_t sck_af;
    GPIO_TypeDef* mosi_port;
    uint16_t mosi_pin;
    uint8_t mosi_af;
    GPIO_TypeDef* dc_port;
    uint16_t dc_pin;
    GPIO_TypeDef* res_port;
    uint16_t res_pin;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    MillisecondTimer* timer;
  };

  explicit ST7735(const Options& opts)
      : opts_(opts), timer_(*opts.timer) {
    InitGpio();
    InitSpi();
    InitDma();
    InitDisplay();
    FillScreen(0x0000);  // Black background
    InitUI();            // Draw static UI elements once
  }

  /// Start a new display update cycle. Call once when new data is ready
  /// (e.g., every 100ms). Then call DisplayPowerStep() every ~10ms.
  void DisplayPowerStart(int32_t voltage_mv, int32_t current_ma) {
    if (render_step_ != 0) return;  // previous cycle still running

    int32_t power_mw = static_cast<int32_t>(
        (static_cast<int64_t>(voltage_mv) * current_ma) / 1000);

    int32_t milli_val;
    switch (update_row_) {
      case 0: milli_val = voltage_mv; break;
      case 1: milli_val = current_ma; break;
      default: milli_val = power_mw; break;
    }
    FormatValue(render_val_, sizeof(render_val_), milli_val);
    render_len_ = 0;
    while (render_val_[render_len_] && render_len_ < 9) render_len_++;
    render_step_ = 1;
  }

  /// Performs one micro-step (~0.5ms). Call every ~10ms.
  /// Returns true when full cycle is complete.
  /// No flicker: characters overwrite in-place with background color,
  /// trailing space cleared only after all chars are drawn.
  bool DisplayPowerStep() {
    if (render_step_ == 0) return true;

    uint8_t char_idx = render_step_ - 1;
    if (char_idx < render_len_) {
      // Draw one character (overwrites old content directly, no flicker)
      DrawCharBold(TEXT_X + char_idx * 18, ROW_Y[update_row_] + 1,
                   render_val_[char_idx], 0xFFFF, 0x0000, 16, 24);
      render_step_++;
      return false;
    }

    // Final step: clear trailing space after last character
    uint16_t trail_x = TEXT_X + render_len_ * 18;
    if (trail_x < UNIT_X - 1) {
      FillRect(trail_x, ROW_Y[update_row_] + 1,
               UNIT_X - 1 - trail_x, CHAR_H, 0x0000);
    }

    // Done: advance to next row
    update_row_ = (update_row_ + 1) % 3;
    render_step_ = 0;
    return true;
  }

  /// Legacy blocking API.
  void DisplayPower(int32_t voltage_mv, int32_t current_ma) {
    DisplayPowerStart(voltage_mv, current_ma);
    while (!DisplayPowerStep()) {}
  }

 private:
  static constexpr uint16_t WIDTH = 160;
  static constexpr uint16_t HEIGHT = 80;

  // Layout constants
  static constexpr uint16_t ROW_Y[3] = {2, 28, 54};
  static constexpr uint16_t ROW_H = 24;
  static constexpr uint16_t BADGE_X = 1;
  static constexpr uint16_t BADGE_W = 24;
  static constexpr uint16_t BADGE_H = 24;
  static constexpr uint16_t TEXT_X = 28;
  static constexpr uint16_t CHAR_W = 16;
  static constexpr uint16_t CHAR_H = 24;
  static constexpr uint16_t CHAR_ADV = 18;
  static constexpr uint16_t BAR_X = 155;
  static constexpr uint16_t BAR_W = 5;
  static constexpr uint16_t UNIT_W = 12;
  static constexpr uint16_t UNIT_H = 18;
  static constexpr uint16_t UNIT_X = BAR_X - UNIT_W - 1;

  // Colors
  static constexpr uint16_t COL_CYAN    = 0x07FF;
  static constexpr uint16_t COL_YELLOW  = 0xFFE0;
  static constexpr uint16_t COL_MAGENTA = 0xF81F;

  // ST7735 internal RAM offsets (132x162, landscape MADCTL=0x60)
  static constexpr uint8_t COL_OFFSET = 1;
  static constexpr uint8_t ROW_OFFSET = 26;

  // Draw all static UI elements (called once from constructor)
  void InitUI() {
    static constexpr uint16_t ROW_COLORS[] = {COL_CYAN, COL_YELLOW, COL_MAGENTA};
    static constexpr char LABELS[] = {'U', 'I', 'P'};
    static constexpr char UNITS[] = {'V', 'A', 'W'};

    for (int i = 0; i < 3; i++) {
      // Badge (rounded rect + label)
      FillRoundRect(BADGE_X, ROW_Y[i], BADGE_W, BADGE_H, 4, ROW_COLORS[i]);
      DrawCharBold(BADGE_X + 6, ROW_Y[i] + 3, LABELS[i],
                   0x0000, ROW_COLORS[i], 12, 18);
      // Unit letter (right before bar)
      DrawCharBold(UNIT_X, ROW_Y[i] + 4, UNITS[i],
                   0xFFFF, 0x0000, UNIT_W, UNIT_H);
      // Colored vertical bar
      FillRect(BAR_X, ROW_Y[i], BAR_W, ROW_H, ROW_COLORS[i]);
    }
  }

  // Format a milli-unit value (e.g., mV, mA, mW) into "X.XXX" string
  void FormatValue(char* buf, int buf_size, int32_t milli_val) {
    int32_t integer = milli_val / 1000;
    int32_t frac = milli_val % 1000;
    if (frac < 0) frac = -frac;
    snprintf(buf, buf_size, "%ld.%03ld",
             static_cast<long>(integer), static_cast<long>(frac));
  }

  // Fill a rectangular area with a solid color
  void FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                uint16_t color) {
    if (w == 0 || h == 0) return;
    SetWindow(x, y, x + w - 1, y + h - 1);
    
    // Fill buffer A with the color
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    for (uint16_t i = 0; i < w; i++) {
      line_buf_a_[i * 2] = hi;
      line_buf_a_[i * 2 + 1] = lo;
    }
    
    DcHigh();
    CsLow();
    for (uint16_t r = 0; r < h; r++) {
      SpiDmaWait();  // ensure previous DMA done before reusing buffer
      SpiDmaStart(line_buf_a_, w * 2, true);
    }
    SpiDmaWait();
    CsHigh();
  }

  // Fill a rounded rectangle (radius r at corners)
  void FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     uint16_t r, uint16_t color) {
    // Clamp radius
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    // Center rectangle (full width, excluding top/bottom radius rows)
    FillRect(x, y + r, w, h - 2 * r, color);
    // Top rectangle (between corners)
    FillRect(x + r, y, w - 2 * r, r, color);
    // Bottom rectangle (between corners)
    FillRect(x + r, y + h - r, w - 2 * r, r, color);

    // Fill four rounded corners using circle quadrant scan
    FillCorner(x + r,         y + r,         r, color, -1, -1);  // top-left
    FillCorner(x + w - r - 1, y + r,         r, color, +1, -1);  // top-right
    FillCorner(x + r,         y + h - r - 1, r, color, -1, +1);  // bottom-left
    FillCorner(x + w - r - 1, y + h - r - 1, r, color, +1, +1);  // bottom-right
  }

  // Fill a quarter-circle corner area
  void FillCorner(int16_t cx, int16_t cy, uint16_t r,
                  uint16_t color, int8_t dx_dir, int8_t dy_dir) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    for (int16_t dy = 0; dy <= (int16_t)r; dy++) {
      for (int16_t dx = 0; dx <= (int16_t)r; dx++) {
        if (dx * dx + dy * dy <= (int16_t)(r * r)) {
          int16_t px = cx + dx * dx_dir;
          int16_t py = cy + dy * dy_dir;
          if (px >= 0 && px < (int16_t)WIDTH &&
              py >= 0 && py < (int16_t)HEIGHT) {
            SetWindow(px, py, px, py);
            DcHigh();
            CsLow();
            SpiSendByte(hi);
            SpiSendByte(lo);
            SpiWaitBsy();
            CsHigh();
          }
        }
      }
    }
  }


  void InitGpio() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio = {};

    // SPI Alternate function pins
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    gpio.Alternate = opts_.sck_af;
    gpio.Pin = opts_.sck_pin;
    HAL_GPIO_Init(opts_.sck_port, &gpio);
    
    gpio.Alternate = opts_.mosi_af;
    gpio.Pin = opts_.mosi_pin;
    HAL_GPIO_Init(opts_.mosi_port, &gpio);

    // Control pins (GPIO)
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Alternate = 0;
    gpio.Pin = opts_.dc_pin;   HAL_GPIO_Init(opts_.dc_port, &gpio);
    gpio.Pin = opts_.res_pin;  HAL_GPIO_Init(opts_.res_port, &gpio);
    gpio.Pin = opts_.cs_pin;   HAL_GPIO_Init(opts_.cs_port, &gpio);
    
    CsHigh();
  }

  void InitSpi() {
    if (opts_.spi == SPI1) {
      __HAL_RCC_SPI1_CLK_ENABLE();
    } else if (opts_.spi == SPI2) {
      __HAL_RCC_SPI2_CLK_ENABLE();
    } else if (opts_.spi == SPI3) {
      __HAL_RCC_SPI3_CLK_ENABLE();
    }

    opts_.spi->CR1 = 0; // Disable SPI
    
    // Master, baud rate fPCLK/4, CPHA=0, CPOL=0
    opts_.spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0; 
    
    // 8 bit mode, transmit only
    opts_.spi->CR2 = (7 << SPI_CR2_DS_Pos) | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH;

    opts_.spi->CR1 |= SPI_CR1_SPE; // Enable SPI
  }

  void InitDma() {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    dmamux_tx_ = Stm32Dma::SelectDmamux(opts_.tx_dma);

    // Common DMA setup
    opts_.tx_dma->CCR =
        DMA_MEMORY_TO_PERIPH |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH |
        DMA_CCR_DIR; // DIR=1 (mem->periph)

    uint32_t req_id = 0;
    if (opts_.spi == SPI1) req_id = DMA_REQUEST_SPI1_TX;
    else if (opts_.spi == SPI2) req_id = DMA_REQUEST_SPI2_TX;
    else if (opts_.spi == SPI3) req_id = DMA_REQUEST_SPI3_TX;

    dmamux_tx_->CCR = req_id & DMAMUX_CxCR_DMAREQ_ID;
    opts_.tx_dma->CPAR = (uint32_t)&opts_.spi->DR;
  }

  void InitDisplay() {
    // Hardware reset
    ResHigh();
    timer_.wait_ms(5);
    ResLow();
    timer_.wait_ms(20);
    ResHigh();
    timer_.wait_ms(150);

    SendCmd(0x01);        // SWRESET
    timer_.wait_ms(150);

    SendCmd(0x11);        // SLPOUT
    timer_.wait_ms(150);

    SendCmd(0xB1);        // FRMCTR1
    SendData(0x01);
    SendData(0x2C);
    SendData(0x2D);

    SendCmd(0xB4);        // INVCTR
    SendData(0x07);

    SendCmd(0xC0);        // PWCTR1
    SendData(0xA2);
    SendData(0x02);
    SendData(0x84);

    SendCmd(0xC1);        // PWCTR2
    SendData(0xC5);

    SendCmd(0xC2);        // PWCTR3
    SendData(0x0A);
    SendData(0x00);

    SendCmd(0xC5);        // VMCTR1
    SendData(0x0E);

    SendCmd(0x21);        // INVON (this panel needs inversion)

    SendCmd(0x3A);        // COLMOD
    SendData(0x05);       // 16-bit color

    SendCmd(0x36);        // MADCTL - landscape
    SendData(0x60);       // MX + MV

    SendCmd(0x13);        // NORON
    timer_.wait_ms(10);

    SendCmd(0x29);        // DISPON
    timer_.wait_ms(100);
  }

  void SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    SendCmd(0x2A);  // CASET
    SendData16(x0 + COL_OFFSET);
    SendData16(x1 + COL_OFFSET);
    SendCmd(0x2B);  // RASET
    SendData16(y0 + ROW_OFFSET);
    SendData16(y1 + ROW_OFFSET);
    SendCmd(0x2C);  // RAMWR
  }

  void FillScreen(uint16_t color) {
    FillRect(0, 0, WIDTH, HEIGHT, color);
  }

  void DrawChar2x(uint16_t x, uint16_t y, char c,
                  uint16_t fg, uint16_t bg) {
    if (c < 0x20 || c > 0x5A) c = ' ';
    const uint8_t* glyph = font5x8::DATA[c - 0x20];
    SetWindow(x, y, x + 11, y + 15);
    DcHigh();
    CsLow();
    uint8_t fg_hi = fg >> 8;
    uint8_t fg_lo = fg & 0xFF;
    uint8_t bg_hi = bg >> 8;
    uint8_t bg_lo = bg & 0xFF;
    
    uint8_t* cur = line_buf_a_;
    uint8_t* nxt = line_buf_b_;
    for (int row = 0; row < 16; row++) {
      int sr = row / 2;
      for (int col = 0; col < 12; col++) {
        int sc = col / 2;
        uint8_t bits = (sc < 5) ? glyph[sc] : 0;
        bool is_fg = (bits & (1 << sr));
        cur[col * 2] = is_fg ? fg_hi : bg_hi;
        cur[col * 2 + 1] = is_fg ? fg_lo : bg_lo;
      }
      SpiDmaWait();
      SpiDmaStart(cur, 12 * 2, true);
      // Swap buffers: CPU fills 'nxt' while DMA sends 'cur'
      uint8_t* tmp = cur; cur = nxt; nxt = tmp;
    }
    SpiDmaWait();
    CsHigh();
  }

  void DrawString2x(uint16_t x, uint16_t y, const char* str,
                    uint16_t fg, uint16_t bg) {
    while (*str) {
      DrawChar2x(x, y, *str, fg, bg);
      x += 12;
      str++;
    }
  }

  // Bold+scaled character from 8x16 Arial font.
  // Scales to out_w x out_h pixels with automatic bold effect.
  // Uses double-buffered DMA pipelining for minimal blocking.
  void DrawCharBold(uint16_t x, uint16_t y, char c,
                    uint16_t fg, uint16_t bg,
                    uint8_t out_w, uint8_t out_h) {
    if (c < 0x20 || c > 0x5A) c = ' ';
    const uint8_t* glyph = font_arial::DATA[c - 0x20];
    SetWindow(x, y, x + out_w - 1, y + out_h - 1);
    DcHigh();
    CsLow();
    uint8_t fg_hi = fg >> 8;
    uint8_t fg_lo = fg & 0xFF;
    uint8_t bg_hi = bg >> 8;
    uint8_t bg_lo = bg & 0xFF;

    uint8_t* cur = line_buf_a_;
    uint8_t* nxt = line_buf_b_;
    for (int row = 0; row < out_h; row++) {
      int sr = row * 16 / out_h;
      uint8_t bits = glyph[sr];
      for (int col = 0; col < out_w; col++) {
        int sc = col * 8 / out_w;
        bool is_fg = (bits & (0x80 >> sc));
        cur[col * 2] = is_fg ? fg_hi : bg_hi;
        cur[col * 2 + 1] = is_fg ? fg_lo : bg_lo;
      }
      SpiDmaWait();
      SpiDmaStart(cur, out_w * 2, true);
      // Swap: CPU fills nxt while DMA sends cur
      uint8_t* tmp = cur; cur = nxt; nxt = tmp;
    }
    SpiDmaWait();
    CsHigh();
  }

  // Default bold string: 14x22 chars, 16px advance
  void DrawStringBold(uint16_t x, uint16_t y, const char* str,
                      uint16_t fg, uint16_t bg) {
    while (*str) {
      DrawCharBold(x, y, *str, fg, bg, 16, 24);
      x += 18;  // 16px char + 2px gap
      str++;
    }
  }

  // --- Low-level SPI ---

  __attribute__((always_inline)) inline void SpiWaitBsy() {
    while ((opts_.spi->SR & SPI_SR_BSY) != 0) {}  
  }
  
  __attribute__((always_inline)) inline void SpiSendByte(uint8_t byte) {
    while ((opts_.spi->SR & SPI_SR_TXE) == 0) {}
    *((volatile uint8_t*)&opts_.spi->DR) = byte;
  }

  /// Start a DMA transfer (non-blocking). Call SpiDmaWait() before
  /// reusing the data buffer or starting the next transfer.
  void SpiDmaStart(const uint8_t* data, uint32_t len, bool minc = true) {
    opts_.tx_dma->CCR &= ~DMA_CCR_EN;
    opts_.tx_dma->CNDTR = len;
    opts_.tx_dma->CMAR = (uint32_t)data;
    if (minc) {
        opts_.tx_dma->CCR |= DMA_CCR_MINC;
    } else {
        opts_.tx_dma->CCR &= ~DMA_CCR_MINC;
    }
    opts_.tx_dma->CCR |= DMA_CCR_EN;
    // Returns immediately – DMA runs in background
  }

  /// Wait for any in-flight DMA + SPI to complete.
  void SpiDmaWait() {
    while (opts_.tx_dma->CNDTR > 0) {}
    SpiWaitBsy();
  }

  /// Blocking DMA send (legacy convenience wrapper).
  void SpiDmaSend(const uint8_t* data, uint32_t len, bool minc = true) {
    SpiDmaStart(data, len, minc);
    SpiDmaWait();
  }

  // SendCmd: keeps CS low, sets DC=low, sends byte
  void SendCmd(uint8_t cmd) {
    DcLow();
    CsLow();
    SpiSendByte(cmd);
    SpiWaitBsy();
    CsHigh();
  }

  // SendData: keeps CS low, sets DC=high, sends byte
  void SendData(uint8_t data) {
    DcHigh();
    CsLow();
    SpiSendByte(data);
    SpiWaitBsy();
    CsHigh();
  }

  void SendData16(uint16_t data) {
    DcHigh();
    CsLow();
    SpiSendByte(data >> 8);
    SpiSendByte(data & 0xFF);
    SpiWaitBsy();
    CsHigh();
  }

  // GPIO via BSRR for speed

  void DcHigh()   { opts_.dc_port->BSRR = opts_.dc_pin; }
  void DcLow()    { opts_.dc_port->BSRR = (uint32_t)opts_.dc_pin << 16; }
  void ResHigh()  { opts_.res_port->BSRR = opts_.res_pin; }
  void ResLow()   { opts_.res_port->BSRR = (uint32_t)opts_.res_pin << 16; }
  void CsHigh()   { opts_.cs_port->BSRR = opts_.cs_pin; }
  void CsLow()    { opts_.cs_port->BSRR = (uint32_t)opts_.cs_pin << 16; }

  Options opts_;
  MillisecondTimer& timer_;
  uint8_t update_row_ = 0;  // cycles 0→1→2→0 for incremental updates
  
  // Micro-step render state
  uint8_t render_step_ = 0;   // 0=idle, 1=clear, 2+=draw char N
  char render_val_[10] = {};   // formatted value string
  uint8_t render_len_ = 0;     // length of render_val_
  
  // Double buffers for DMA pipelining
  uint8_t line_buf_a_[WIDTH * 2];
  uint8_t line_buf_b_[WIDTH * 2];
  DMAMUX_Channel_TypeDef* dmamux_tx_ = nullptr;
};

}  // namespace fw
