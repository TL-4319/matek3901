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
  inline int32_t range_mm() const {return range_mm_.i4;}
  inline uint8_t sur_qual() const {return sur_qual_;}
  inline uint8_t range_qual() const {return range_qual_;}
  inline int32_t mot_x() const {return mot_x_.i4;}
  inline int32_t mot_y() const {return mot_y_.i4;}
  
 private:
 /* Checksum */
  void crc8_dvb_s2(uint8_t &crc, const uint8_t &a);
  /* Communication */
  static constexpr int16_t COMM_TIMEOUT_MS_ = 5000;
  static constexpr int32_t BAUD_ = 115200;
  HardwareSerial *bus_;
  elapsedMillis t_ms_;
  /* Data */
  uint8_t sur_qual_;
  uint8_t range_qual_;
  typedef union {uint8_t bytes[4]; int32_t i4;} union_32;
  union_32 range_mm_, mot_x_, mot_y_; 
  /* Parser */
  static constexpr uint8_t MATEK_HEADER1_ = 0x24;
  static constexpr uint8_t MATEK_HEADER2_ = 0x58;
  static constexpr uint8_t MATEK_TYPE_ = 0x3C;
  static constexpr uint8_t MATEK_FLAG_ = 0x00;
  //static constexpr uint8_t MATEK_FUNC_CLASS_ = 0x1F;
  static constexpr uint8_t MATEK_OPFLOW_ = 0x02;
  static constexpr uint8_t MATEK_RANGE_ = 0x01;

  static constexpr uint8_t HEADER1_POS_ = 0;
  static constexpr uint8_t HEADER2_POS_ = 1;
  static constexpr uint8_t TYPE_POS_ = 2;
  static constexpr uint8_t FLAG_POS_ = 3;
  static constexpr uint8_t SENSOR_TYPE_POS_ = 4;
  static constexpr uint8_t LIDAR_CHECKSUM_POS_ = 13;
  static constexpr uint8_t OPFLOW_CHECKSUM_POS_ = 17;

  /* Driver state parameters*/
  uint8_t c_;
  uint8_t state_ = 0;
  uint8_t buf_[13];
  uint8_t type_ = 0; // 0 - None, 1 - lidar, 2 - opflow
  uint8_t crc_ = 0x00;
};

}  // namespace bfs

#endif  // AINSTEIN_USD1_SRC_AINSTEIN_USD1_H_ NOLINT