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
        continue;
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == HEADER2_POS_) {
      if (c_ == MATEK_HEADER2_) {
        state_++;
        continue;
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == TYPE_POS_) {
      if (c_ == MATEK_TYPE_) {
        state_++;
        continue;
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == FLAG_POS_) {
      if (c_ == MATEK_FLAG_) {
        state_++;
        continue;
      }
      else{
        state_ = 0;
      }
    }
    else if (state_ == SENSOR_TYPE_POS_) {
      if (c_ == MATEK_OPFLOW_) {
        msg_type_ = OPFLOW;
        state_++;
        continue;
      }
      else if (c_ == MATEK_RANGE_) {
        msg_type_ = RANGE;
        state_++;
        continue;
      }
      else{
        state_ = 0;
      }
    }
    

    // Handle msg from different sensors
    if (msg_type_ == RANGE) {
      if (state_ == FUNC_POS_){
        if (c_ = MATEK_FUNC_CLASS_){
          state_++;
        }
      }
      else if (state_ == SIZE_LSB_POS_){
        if (c_ == 0x05){
          size_lsb_ = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if (state_ == SIZE_MSB_POS_){
        if (c_ == 0x00){
          size_msb_ = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if ((state_ >= RANGE_POS_) && (state_ < RANGE_QUAL_POS_)){
        range_buf_[state_ - RANGE_POS_] = c_;
        state_++;
      }
      else if (state_ == RANGE_QUAL_POS_){
        range_qual_ = c_;
        state_++;
      }
      else if (state_ == RANGE_CHK_POS_){
        chk_ = c_;
        /* DO CHECKSUM AND DATA CALCULATION HERE BOI */

        //
        state_ = 0;
        return true;
      }
    }
    else if (msg_type_ == OPFLOW) {
      if (state_ == FUNC_POS_){
        if (c_ = MATEK_FUNC_CLASS_){
          state_++;
        }
      }
      else if (state_ == SIZE_LSB_POS_){
        if (c_ == 0x09){
          size_lsb_ = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if (state_ == SIZE_MSB_POS_){
        if (c_ == 0x00){
          size_msb_ = c_;
          state_ ++;
        }
        else {
          state_ = 0;
        }
      }
      else if ((state_ >= XMOT_POS_) && (state_ < YMOT_POS_)){
        xmot_buf_[state_ - XMOT_POS_] = c_;
        state_++;
      }
      else if ((state_ >= YMOT_POS_) && (state_ < OPFLOW_QUAL_POS_)){
        ymot_buf_[state_ - YMOT_POS_] = c_;
        state_++;
      }
      else if (state_ == OPFLOW_QUAL_POS_){
        sur_qual_ = c_;
        state_++;
      }
      else if (state_ == OPFLOW_CHK_POS_){
        chk_ = c_;
        /* DO CHECKSUM AND DATA CALCULATION HERE BOI */

        //
        new_flow_data = true
        state_ = 0;
        return true;
      }
    }

  }
  return false;
}

}  // namespace bfs
