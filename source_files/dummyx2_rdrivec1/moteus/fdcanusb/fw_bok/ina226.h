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
#include "mbed.h"

namespace fw {

/// Software (bit-bang) I2C master using direct GPIO register access.
/// Configured for open-drain with external pull-ups.
class SoftI2C {
 public:
  SoftI2C(GPIO_TypeDef* sda_port, uint16_t sda_pin,
          GPIO_TypeDef* scl_port, uint16_t scl_pin)
      : sda_port_(sda_port), sda_pin_(sda_pin),
        scl_port_(scl_port), scl_pin_(scl_pin) {
    Init();
  }

  bool WriteRegister16(uint8_t addr7, uint8_t reg, uint16_t value) {
    Start();
    if (!WriteByte(addr7 << 1)) { Stop(); return false; }
    if (!WriteByte(reg)) { Stop(); return false; }
    if (!WriteByte(value >> 8)) { Stop(); return false; }
    if (!WriteByte(value & 0xFF)) { Stop(); return false; }
    Stop();
    return true;
  }

  bool ReadRegister16(uint8_t addr7, uint8_t reg, uint16_t* value) {
    // Write register address
    Start();
    if (!WriteByte(addr7 << 1)) { Stop(); return false; }
    if (!WriteByte(reg)) { Stop(); return false; }
    // Repeated start + read
    Start();
    if (!WriteByte((addr7 << 1) | 1)) { Stop(); return false; }
    uint8_t hi = ReadByte(true);   // ACK
    uint8_t lo = ReadByte(false);  // NACK (last byte)
    Stop();
    *value = (static_cast<uint16_t>(hi) << 8) | lo;
    return true;
  }

 private:
  void Init() {
    // Enable GPIO clocks for the ports we use.
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};

    // SDA pin: open-drain output, no internal pull-up (external 10K exists)
    gpio.Pin = sda_pin_;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sda_port_, &gpio);

    // SCL pin: open-drain output, no internal pull-up (external 10K exists)
    gpio.Pin = scl_pin_;
    HAL_GPIO_Init(scl_port_, &gpio);

    // Start with both lines released (high)
    SdaHigh();
    SclHigh();
    DelayUs(10);
  }

  void Start() {
    SdaHigh();
    SclHigh();
    DelayUs(5);
    SdaLow();
    DelayUs(5);
    SclLow();
    DelayUs(5);
  }

  void Stop() {
    SdaLow();
    SclHigh();
    DelayUs(5);
    SdaHigh();
    DelayUs(5);
  }

  bool WriteByte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
      if (byte & (1 << i)) {
        SdaHigh();
      } else {
        SdaLow();
      }
      DelayUs(2);
      SclHigh();
      DelayUs(5);
      SclLow();
      DelayUs(2);
    }
    // Read ACK
    SdaHigh();
    DelayUs(2);
    SclHigh();
    DelayUs(5);
    bool ack = !SdaRead();
    SclLow();
    DelayUs(2);
    return ack;
  }

  uint8_t ReadByte(bool ack) {
    SdaHigh();
    uint8_t byte = 0;
    for (int i = 7; i >= 0; i--) {
      DelayUs(2);
      SclHigh();
      DelayUs(5);
      if (SdaRead()) {
        byte |= (1 << i);
      }
      SclLow();
      DelayUs(2);
    }
    if (ack) {
      SdaLow();
    } else {
      SdaHigh();
    }
    DelayUs(2);
    SclHigh();
    DelayUs(5);
    SclLow();
    DelayUs(2);
    SdaHigh();
    return byte;
  }

  void SdaHigh() { HAL_GPIO_WritePin(sda_port_, sda_pin_, GPIO_PIN_SET); }
  void SdaLow()  { HAL_GPIO_WritePin(sda_port_, sda_pin_, GPIO_PIN_RESET); }
  void SclHigh() { HAL_GPIO_WritePin(scl_port_, scl_pin_, GPIO_PIN_SET); }
  void SclLow()  { HAL_GPIO_WritePin(scl_port_, scl_pin_, GPIO_PIN_RESET); }
  bool SdaRead() { return HAL_GPIO_ReadPin(sda_port_, sda_pin_) == GPIO_PIN_SET; }

  void DelayUs(uint32_t us) {
    const uint32_t cycles = us * (SystemCoreClock / 1000000);
    volatile uint32_t count = cycles / 4;
    while (count--) { __NOP(); }
  }

  GPIO_TypeDef* sda_port_;
  uint16_t sda_pin_;
  GPIO_TypeDef* scl_port_;
  uint16_t scl_pin_;
};


/// INA226 power monitor driver.
///
/// Reads bus voltage (1.25 mV/LSB) and shunt voltage (2.5 uV/LSB).
/// Current is calculated from shunt voltage and the known shunt resistance.
class INA226 {
 public:
  struct Options {
    GPIO_TypeDef* sda_port;
    uint16_t sda_pin;
    GPIO_TypeDef* scl_port;
    uint16_t scl_pin;
    uint8_t i2c_addr;
    uint32_t shunt_resistance_uohm;
  };

  explicit INA226(const Options& options)
      : i2c_(options.sda_port, options.sda_pin,
             options.scl_port, options.scl_pin),
        addr_(options.i2c_addr),
        shunt_r_uohm_(options.shunt_resistance_uohm) {
    // Configure INA226:
    //   Averaging = 16 samples (bits 11:9 = 010)
    //   Bus voltage conversion time = 1.1ms (bits 8:6 = 100)
    //   Shunt voltage conversion time = 1.1ms (bits 5:3 = 100)
    //   Mode = shunt and bus, continuous (bits 2:0 = 111)
    const uint16_t config = 0x4427;
    i2c_.WriteRegister16(addr_, REG_CONFIG, config);
  }

  /// Call this periodically (e.g., every 100 ms).
  void Poll() {
    uint16_t raw_bus = 0;
    uint16_t raw_shunt = 0;

    if (i2c_.ReadRegister16(addr_, REG_BUS_VOLTAGE, &raw_bus)) {
      // Bus voltage LSB = 1.25 mV
      voltage_mv_ = static_cast<int32_t>(raw_bus) * 125 / 100;
    }

    if (i2c_.ReadRegister16(addr_, REG_SHUNT_VOLTAGE, &raw_shunt)) {
      // Shunt voltage register is signed, LSB = 2.5 uV
      int16_t signed_shunt = static_cast<int16_t>(raw_shunt);
      // current_mA = signed_shunt * 2.5uV / (R_uohm / 1000)
      //            = signed_shunt * 2500 / R_uohm
      current_ma_ = static_cast<int32_t>(signed_shunt) * 2500 /
                    static_cast<int32_t>(shunt_r_uohm_);
    }
  }

  int32_t voltage_mv() const { return voltage_mv_; }
  int32_t current_ma() const { return current_ma_; }

 private:
  static constexpr uint8_t REG_CONFIG        = 0x00;
  static constexpr uint8_t REG_SHUNT_VOLTAGE = 0x01;
  static constexpr uint8_t REG_BUS_VOLTAGE   = 0x02;

  SoftI2C i2c_;
  uint8_t addr_;
  uint32_t shunt_r_uohm_;

  int32_t voltage_mv_ = 0;
  int32_t current_ma_ = 0;
};

}  // namespace fw
