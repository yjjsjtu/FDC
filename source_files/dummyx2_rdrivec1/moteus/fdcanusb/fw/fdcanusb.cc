// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include <inttypes.h>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/string_span.h"

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/can_manager.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_async_uart.h"
#include "fw/stm32g4_flash.h"
#include "fw/stm32g4_async_usb_cdc.h"
#include "fw/stm32g4_gs_usb.h"
#include "fw/ina226.h"
#include "fw/st7735.h"
#include "fw/uuid.h"
#include "fw/xprintf.h"

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;

void SetupClock(int clock_rate_hz) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  // Temporarily stop running off the PLL so we can change it.
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = (clock_rate_hz / 1000000) * 24 / 48;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    mbed_die();
  }

  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }
}

class ClockManager {
 public:
  ClockManager(fw::MillisecondTimer* timer,
               micro::PersistentConfig& persistent_config,
               micro::CommandManager& command_manager)
      : timer_(timer) {
    persistent_config.Register("clock", &clock_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("clock", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  int32_t clock_hz() const {
    return clock_.can_hz;
  }

  void UpdateConfig() {
    const int can_clock_hz = [&]() {
      if (clock_.can_hz >= 85000000) { return 85000000; }
      if (clock_.can_hz >= 80000000) { return 80000000; }
      if (clock_.can_hz >= 60000000) { return 60000000; }
      return 85000000;
    }();

    SetupClock(can_clock_hz * 2);

    const int32_t trim = std::max<int32_t>(0, std::min<int32_t>(127, clock_.hsitrim));
    RCC->ICSCR = (RCC->ICSCR & ~0xff000000) | (trim << 24);
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    if (command == "us") {
      snprintf(output_, sizeof(output_), "%" PRIu32 "\r\n", timer_->read_us());
      WriteMessage(output_, response);
    } else {
      WriteMessage("ERR unknown clock\r\n", response);
    }
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

 private:
  struct Config {
    int32_t can_hz = 85000000;
    int32_t hsitrim = 64;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(can_hz));
      a->Visit(MJ_NVP(hsitrim));
    }
  };

  fw::MillisecondTimer* const timer_;
  Config clock_;
  char output_[16] = {};
};

struct LcdConfig {
  // 0 = off, 1-100 = brightness percentage
  int32_t brightness = 100;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(brightness));
  }
};

/// PWM backlight driver using TIM4_CH2 on PB7 (AF2).
class PwmBacklight {
 public:
  PwmBacklight() {
    // Enable TIM4 and GPIOB clocks
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure PB7 as TIM4_CH2 alternate function
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    // TIM4: 170 MHz / 170 = 1 MHz tick, period 1000 => 1 kHz PWM
    htim_.Instance = TIM4;
    htim_.Init.Prescaler = 169;       // 170MHz / (169+1) = 1 MHz
    htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_.Init.Period = 999;          // 1 MHz / (999+1) = 1 kHz
    htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim_);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 999;                   // default 100% duty
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim_, &oc, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim_, TIM_CHANNEL_2);
  }

  /// Set brightness 0-100 (percentage).
  void set_brightness(int32_t pct) {
    if (pct <= 0) {
      __HAL_TIM_SET_COMPARE(&htim_, TIM_CHANNEL_2, 0);
    } else if (pct >= 100) {
      __HAL_TIM_SET_COMPARE(&htim_, TIM_CHANNEL_2, 999);
    } else {
      // Map 1-99 => 10-999 (avoid barely-visible flicker at very low values)
      const uint32_t compare = 10 + static_cast<uint32_t>(pct) * 989 / 100;
      __HAL_TIM_SET_COMPARE(&htim_, TIM_CHANNEL_2, compare);
    }
  }

 private:
  TIM_HandleTypeDef htim_ = {};
};

}

int main(void) {
  SetupClock(170000000);

  DigitalOut power_led{PB_5, 1};

  fw::MillisecondTimer timer;

  micro::SizedPool<20480> pool;

  fw::Stm32G4AsyncUsbCdc usb(
      &pool,
      [&]() {
        fw::Stm32G4AsyncUsbCdc::Options options;
        options.timer = &timer;
        return options;
      }());

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&usb);
  micro::CommandManager command_manager(
      &pool, &usb, &write_stream,
      []() {
        micro::CommandManager::Options options;
        options.max_line_length = 300;
        return options;
      }());

  char micro_output_buffer[2048] = {};
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream, micro_output_buffer);

  fw::Stm32G4Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface, micro_output_buffer);

  fw::Uuid uuid(persistent_config);
  ClockManager clock(&timer, persistent_config, command_manager);

  fw::CanManager can_manager(
      pool, persistent_config, command_manager, write_stream,
      [&]() {
        fw::CanManager::Options options;
        options.td = PB_13;
        options.rd = PB_12;
        options.cdc = &usb;
        return options;
      }());

  fw::Stm32G4GsUsb gs_usb(pool, can_manager, [&]() {
    fw::Stm32G4GsUsb::Options options;
    options.get_can_clock_hz = [&]() { return clock.clock_hz(); };
    options.power_led = &power_led;
    options.timer = &timer;
    return options;
  }());

  usb.RegisterGsUsbHandler([&gs_usb](auto* dev, auto* req, auto* callback) {
    return gs_usb.HandleControl(dev, req, callback);
  });
  usb.RegisterGsUsbEndpointHandlers(
      [&gs_usb](usbd_device* dev, uint8_t event, uint8_t ep) {
        gs_usb.HandleRxEndpoint(dev, event, ep);
      },
      [&gs_usb](usbd_device* dev, uint8_t event, uint8_t ep) {
        gs_usb.HandleTxEndpoint(dev, event, ep);
      });
  can_manager.RegisterFrameCallback(
      [&gs_usb](const fw::CanManager::CanFrame& frame) {
        gs_usb.OnCanFrameReceived(frame);
      });

  // INA226 power monitor on I2C (PF0=SDA, PC4=SCL)
  fw::INA226 ina226([&]() {
    fw::INA226::Options options;
    options.sda_port = GPIOF;
    options.sda_pin = GPIO_PIN_0;
    options.scl_port = GPIOC;
    options.scl_pin = GPIO_PIN_4;
    options.i2c_addr = 0x40;             // A0=GND, A1=GND
    options.shunt_resistance_uohm = 1000; // 1 mohm = 1000 uohm
    return options;
  }());

  // ST7735 160x80 LCD on hardware SPI + DMA
  fw::ST7735 lcd([&]() {
    fw::ST7735::Options options;
    options.spi = SPI1;
    options.tx_dma = DMA1_Channel3;
    options.sck_port = GPIOA;
    options.sck_pin = GPIO_PIN_5;
    options.sck_af = GPIO_AF5_SPI1;
    options.mosi_port = GPIOA;
    options.mosi_pin = GPIO_PIN_7;
    options.mosi_af = GPIO_AF5_SPI1;
    options.dc_port = GPIOA;
    options.dc_pin = GPIO_PIN_8;
    options.res_port = GPIOA;
    options.res_pin = GPIO_PIN_9;
    options.cs_port = GPIOA;
    options.cs_pin = GPIO_PIN_10;
    options.timer = &timer;
    return options;
  }());

  // LCD backlight PWM on PB7 (TIM4_CH2)
  PwmBacklight lcd_backlight;

  fw::FirmwareInfo firmware_info(pool, telemetry_manager);

  fw::GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  LcdConfig lcd_config;
  persistent_config.Register("lcd", &lcd_config, [&](){
    lcd_backlight.set_brightness(lcd_config.brightness);
  });

  persistent_config.Load();
  lcd_backlight.set_brightness(lcd_config.brightness);

  command_manager.AsyncStart();
  can_manager.Start();

#ifdef MJBOTS_XPRINTF_DEBUG
  fw::debug_print_init();
#endif

  auto old_time_ms = timer.read_ms();
  int tenms_count = 0;
  int hundredms_count = 0;
  uint32_t loop_count = 0;

  while (true) {
    loop_count++;
#ifdef MJBOTS_XPRINTF_DEBUG
    fw::debug_print_poll();
#endif
    can_manager.Poll();
    usb.Poll();
    gs_usb.Poll();

    const auto new_time = timer.read_ms();
    if (new_time != old_time_ms) {
      old_time_ms = new_time;

      can_manager.PollMillisecond();
      gs_usb.PollMillisecond();

      tenms_count++;
      if (tenms_count >= 10) {
        can_manager.Poll10Ms();
        usb.Poll10Ms();

        // INA226: one micro-step per 10ms tick (~200us each)
        if (lcd_config.brightness > 0) {
          ina226.PollStep();
          // LCD: one micro-step per 10ms (~0.5ms each: clear or 1 char)
          lcd.DisplayPowerStep();
        }

        // Start new LCD row render + update power info every 100ms
        hundredms_count++;
        if (hundredms_count >= 10) {
          hundredms_count = 0;
          if (lcd_config.brightness > 0) {
            fw::CanManager::PowerInfo pi;
            pi.voltage_mv = ina226.voltage_mv();
            pi.current_ma = ina226.current_ma();
            can_manager.SetPowerInfo(pi);

            // Kick off next row render (will be drawn over next ~80ms)
            lcd.DisplayPowerStart(ina226.voltage_mv(), ina226.current_ma());
          }
        }

#ifndef MJBOTS_XPRINTF_DEBUG
        // To verify xprintf works:
        xprintf("loop_count %d\r\n", loop_count);
#endif

        tenms_count = 0;

      }
    }
  }
}

extern "C" {
void SysTick_Handler(void) {
  HAL_IncTick();
}

void abort() {
  mbed_die();
}
}

