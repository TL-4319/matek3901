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

uint8_t Matek3901::checksum(const uint8_t &buf[14], const uint8_t &size){
  // Implement crc8_dvb_s2 checksum
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

bool Matek3901::Read() {
  bool new_flow_data = false;
  while ((bus_->available()) && !new_flow_data) {
    c_ = bus_->read();
    // Parse message until the point the message type 
    if (state_ == HEADER1_POS_) {
      if (c_ == MATEK_HEADER1_) {
        state_++;
        
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == HEADER2_POS_) {
      if (c_ == MATEK_HEADER2_) {
        state_++;
        
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == TYPE_POS_) {
      if (c_ == MATEK_TYPE_) {
        state_++;
        
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == FLAG_POS_) {
      if (c_ == MATEK_FLAG_) {
        state_++;
        chk_buf_[0] = c_;
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == SENSOR_TYPE_POS_) {
      if (c_ == MATEK_OPFLOW_) {
        msg_type_ = OPFLOW;
        chk_buf_[1] = c_;
        state_++;
        
      }
      else if (c_ == MATEK_RANGE_) {
        msg_type_ = RANGE;
        chk_buf_[1] = c_;
        state_++;
        
      }
      else{
        state_ = 0;
      }
    }
    

    // Handle msg from different sensors
    if (msg_type_ == RANGE) {
      if (state_ == FUNC_POS_){
        if (c_ = MATEK_FUNC_CLASS_){
          chk_buf_[2] = c_;
          state_++;
        }
      }
      else if (state_ == SIZE_LSB_POS_){
        if (c_ == 0x05){
          size_lsb_ = c_;
          chk_buf_[3] = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if (state_ == SIZE_MSB_POS_){
        if (c_ == 0x00){
          size_msb_ = c_;
          chk_buf_[3] = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if ((state_ >= RANGE_POS_) && (state_ < RANGE_QUAL_POS_)){
        range_buf_[state_ - RANGE_POS_] = c_;
        chk_buf_[state_ - 3] = c_;
        state_++;
      }
      else if (state_ == RANGE_QUAL_POS_){
        range_qual_buf_ = c_;
        chk_buf_[9] = c_;
        state_++;
      }
      else if (state_ == RANGE_CHK_POS_){
        chk_ = checksum(chk_buf_, 10);
        if (c_ == chk_){
          range_qual_ = range_qual_buf;
          range_mm_ = (int32_t)(((uint32_t)range_buf_[3]<<24) | ((uint32_t)range_buf_[2]<<16) |
                                ((uint32_t)range_buf_[1]<<8) | range_buf_[0]);
          range_m_ = float(range_mm)_  * mm2m;
        }
        state_ = 0;
      }
    }
    else if (msg_type_ == OPFLOW) {
      if (state_ == FUNC_POS_){
        if (c_ = MATEK_FUNC_CLASS_){
          chk_buf_[2] = c_;
          state_++;
        }
      }
      else if (state_ == SIZE_LSB_POS_){
        if (c_ == 0x09){
          size_lsb_ = c_;
          chk_buf_[3] = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if (state_ == SIZE_MSB_POS_){
        if (c_ == 0x00){
          size_msb_ = c_;
          chk_buf_[4] = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if ((state_ >= XMOT_POS_) && (state_ < YMOT_POS_)){
        xmot_buf_[state_ - XMOT_POS_] = c_;
        chk_buf_[state_ - 3] = c_;
        state_++;
      }
      else if ((state_ >= YMOT_POS_) && (state_ < OPFLOW_QUAL_POS_)){
        ymot_buf_[state_ - YMOT_POS_] = c_;
        chk_buf_[state_ - 3] = c_;
        state_++;
      }
      else if (state_ == OPFLOW_QUAL_POS_){
        sur_qual_buf_ = c_;
        chk_buf_[13] = c_;
        state_++;
      }
      else if (state_ == OPFLOW_CHK_POS_){
        chk_ = checksum(chk_buf_, 14);
        if (c_ == chk_){
          sur_qual_ = sur_qual_buf;
          x_mot_ = (int32_t)(((uint32_t)xmot_buf_[3]<<24) | ((uint32_t)xmot_buf_[2]<<16) |
                                ((uint32_t)xmot_buf_[1]<<8) | xmot_buf_[0]);
          y_mot_ = (int32_t)(((uint32_t)ymot_buf_[3]<<24) | ((uint32_t)ymot_buf_[2]<<16) |
                                ((uint32_t)ymot_buf_[1]<<8) | ymot_buf_[0]);
        }
        new_flow_data = true
        state_ = 0;
        return true;
      }
    }

  }
  return false;
}

}  // namespace bfs
