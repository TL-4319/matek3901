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
  new_flow_data = false;
  while ((bus_->available()) && !new_flow_data) {
    c_ = bus_->read();
    // Parse message until the point the message type 
    if (state_ == HEADER1_POS_) {
      if (c_ == MATEK_HEADER1_) {
        state_++;
      }
    }
    else if (state_ == HEADER2_POS_) {
      if (c_ == MATEK_HEADER2_) {
        state_++;
      }
    }
    else if (state_ == TYPE_POS_) {
      if (c_ == MATEK_TYPE_) {
        state_++;
      }
    }
    else if (state_ == FLAG_POS_) {
      if (c_ == MATEK_FLAG_) {
        state_++;
      }
    }
    else if (state_ == FUNC_POS_) {
      if (c_ == MATEK_FUNC_CLASS_) {
        state_++;
      }
    }
    else if (state_ == SENSOR_TYPE_POS_) {
      if (c_ == MATEK_OPFLOW_) {
        msg_type_ = OPFLOW;
        state_++;
      }
      else if (c_ == MATEK_RANGE_) {
        msg_type_ = RANGE;
        state_++;
      }
    }
    else if (state_ == SIZE_LSB_POS_) {
      size_lsb_ = c_;
      state_++;
    }
    else if (state_ == SIZE_MSB_POS_) {
      size_msb_ = c_;
      state_++;
    }
    else {
      state_ = 0;
    }

    // Handle msg from different sensors
    if (msg_type_ == RANGE) {
    

    }
    else if (msg_type_ == OPFLOW) {


    }

    switch (state_) {
      case HEADER_POS_: {
        if (c_ == HEADER_) {
          state_++;
        }
        break;
      }
      case VER_POS_: {
        if (c_ == VER_) {
          state_++;
        }
        break;
      }
      case ALT_LSB_POS_: {
        alt_lsb_ = c_;
        state_++;
        break;
      }
      case ALT_MSB_POS_: {
        alt_msb_ = c_;
        state_++;
        break;
      }
      case SNR_POS_: {
        snr_ = c_;
        state_++;
        break;
      }
      case CHK_POS_: {
        state_ = 0;
        /* Compute and check the checksum */
        chk_ = (VER_ + alt_msb_ + alt_lsb_ + snr_) & 0xFF;
        if (chk_ == c_) {
          alt_cm_ = static_cast<int16_t>(alt_msb_) << 8 | alt_lsb_;
          alt_m_ = static_cast<float>(alt_cm_) * cm2m;
          snr_nd_ = snr_;
          return true;
        }
        break;
      }
      default: {
        state_ = 0;
        break;
      }
    }
  }
  return false;
}

}  // namespace bfs
