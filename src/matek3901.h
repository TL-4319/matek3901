/*
* Tuan Luong
* tdluong@crimson.ua.edu
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef MATEK3901_SRC_MATEK3901_H_  // NOLINT
#define MATEK3901_SRC_MATEK3901_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif

namespace bfs {

class Matek3901 {
 public:
  Matek3901() {}
  explicit Matek3901(HardwareSerial *bus) : bus_(bus) {}
  void Config(HardwareSerial *bus) {bus_ = bus;}
  bool Begin();
  bool Read();
  inline float range_m() const {return range_m_;}
  inline uint8_t sur_qual() const {return sur_qual_;}
  inline uint8_t range_qual() const {return range_qual_;}
  inline int32_t x_mot() const {return x_mot_;}
  inline int32_t y_mot() const {return y_mot_;}

 private:
  /* Checksum */
  //uint8_t checksum()
  /* Communication */
  static constexpr int16_t COMM_TIMEOUT_MS_ = 5000;
  static constexpr int32_t BAUD_ = 115200;
  HardwareSerial *bus_;
  elapsedMillis t_ms_;
  /* Data */
  uint8_t sur_qual_;
  uint8_t range_qual_;
  int32_t x_mot_;
  int32_t y_mot_;
  uint32_t range_mm_;
  float range_m_;
  static constexpr float mm2m = 1.0f / 1000.0f;
  /* Parser */
  static constexpr uint8_t MATEK_HEADER1_ = 0x24;
  static constexpr uint8_t MATEK_HEADER2_ = 0x58;
  static constexpr uint8_t MATEK_TYPE_ = 0x3C;
  static constexpr uint8_t MATEK_FLAG_ = 0x00;
  static constexpr uint8_t MATEK_FUNC_CLASS_ = 0x1F;
  static constexpr uint8_t MATEK_OPFLOW_ = 0x02;
  static constexpr uint8_t MATEK_RANGE_ = 0x01;

  static constexpr uint8_t HEADER1_POS_ = 0;
  static constexpr uint8_t HEADER2_POS_ = 1;
  static constexpr uint8_t TYPE_POS_ = 2;
  static constexpr uint8_t FLAG_POS_ = 3;
  static constexpr uint8_t SENSOR_TYPE_POS_ = 4;
  static constexpr uint8_t FUNC_POS_ = 5;
  static constexpr uint8_t SIZE_LSB_POS_ = 6;
  static constexpr uint8_t SIZE_MSB_POS_ = 7;
  /* RANGE FRAME */
  static constexpr uint8_t RANGE_POS_ = 8;
  static constexpr uint8_t RANGE_QUAL_POS_ = 12;
  static constexpr uint8_t RANGE_CHK_POS_ = 13;
  /* OPFLOW FRAME */
  static constexpr uint8_t XMOT_POS_ = 8;
  static constexpr uint8_t YMOT_POS_ = 12;
  static constexpr uint8_t OPFLOW_QUAL_POS_ = 16;
  static constexpr uint8_t OPFLOW_CHK_POS_ = 17;

  enum msg_type {
    RANGE,
    OPFLOW
  } msg_type_;
  uint8_t c_;
  uint8_t state_ = 0;
  uint8_t sur_qual_, range_qual_, chk_, size_lsb_, size_msb_;
  uint8_t buf_[6];
  uint8_t range_buf_[4];
  uint8_t xmot_buf_[4];
  uint8_t ymot_buf_[4];
};

}  // namespace bfs

#endif  // AINSTEIN_USD1_SRC_AINSTEIN_USD1_H_ NOLINT
