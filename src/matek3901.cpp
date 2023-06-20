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

#include "matek3901.h"  // NOLINT
#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif

namespace bfs {

void Matek3901::crc8_dvb_s2(uint8_t &crc, const uint8_t &a){
  crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
}

bool Matek3901::Begin() {
  bus_->end();
  bus_->begin(BAUD_);
  bus_->flush();
  t_ms_ = 0;
  while (t_ms_ < COMM_TIMEOUT_MS_) {
    if (Read()) {
      return true;
    }
  }
  return false;
}

bool Matek3901::Read(){
  while (bus_->available()) {
    c_ = bus_->read();
    if (state_ == HEADER1_POS_) {
      if (c_ == MATEK_HEADER1_) {
        state_++;
      }
      else {
        state_ = 0;
      }
    }

    else if (state_ == HEADER2_POS_) {
      if (c_ == MATEK_HEADER2_) {
        state_++; 
      }
      else {
        state_ = 0;
      }
    }

    else if (state_ == TYPE_POS_) {
      if (c_ == MATEK_TYPE_) {
        state_++;
      }
      else {
        state_ = 0;
      }
    }

    else if (state_ == FLAG_POS_) {
      if (c_ == MATEK_FLAG_) {
        state_++;
      }
      else {
        state_ = 0;
      }
    }

    else if (state_ == SENSOR_TYPE_POS_){
      if (c_ == MATEK_RANGE_){
        state_++;
        type_ = 1;
        crc_ = 0xD5;
      }
      else if (c_ == MATEK_OPFLOW_){
        state_++;
        type_ = 2;
        crc_ = 0x7F;
      }
      else {
        state_ = 0;
        type_ = 0;
      }
    }

    // Lidar payload frame
    else if (type_ == 1 && state_ > SENSOR_TYPE_POS_ && state_ < LIDAR_CHECKSUM_POS_){
      buf_[state_ - SENSOR_TYPE_POS_ - 1] = c_;
      crc8_dvb_s2(crc_, c_);
      state_++;
    }
    // Lidar checksum
    else if (type_ == 1 && state_ == LIDAR_CHECKSUM_POS_){
      if (crc_ == c_){
        for (uint8_t i = 0; i < 4; i++){
          range_mm_.bytes[i] = buf_[4+i];
        }
        range_qual_ = buf_[3];
      }
      state_ = 0;
      type_ = 0;
    }

    // Opflow payload frame
    else if (type_ == 2 && state_ > SENSOR_TYPE_POS_ && state_ < OPFLOW_CHECKSUM_POS_){
      buf_[state_ - SENSOR_TYPE_POS_ - 1] = c_;
      crc8_dvb_s2(crc_, c_);
      state_++;
    }
    // Opflow checksum
    else if (type_ == 2 && state_ == OPFLOW_CHECKSUM_POS_){
      if (crc_ == c_){
        sur_qual_ = buf_[3];
          for(uint8_t i = 0; i < 4; i++){
            mot_x_.bytes[i] = buf_[4+i]; 
            mot_y_.bytes[i] = buf_[8+i];
          }
        state_ = 0;
        type_ = 0;
        return true;
      }
        state_ = 0;
        type_ = 0;
      }
    }
    return false; 
  }
} // namespace bfs