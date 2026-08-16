// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "sbgc_driver/sbgc_link.hpp"

#include <cstring>
#include <string>

#include "sbgc_gui_config.h"  // NOLINT(build/include_subdir)

namespace sbgc_driver
{

SbgcLink::SbgcLink()
{
  sb_.fd = -1;
}

SbgcLink::~SbgcLink() {close();}

std::string SbgcLink::discoverPort(const std::string & requested)
{
  char resolved[256];
  if (sbgc_gui_config_resolve_port(
      requested.empty() ? nullptr : requested.c_str(),
      resolved, sizeof(resolved)) == 0)
  {
    return std::string(resolved);
  }

  if (!requested.empty()) {
    // A named port that could not be resolved is reported as asked for, so the
    // failure names the device the operator actually configured.
    return requested;
  }

  sbgc_gui_config_t cfg{};
  if (sbgc_gui_config_discover(&cfg, nullptr) == 0 && cfg.have_port) {
    return std::string(cfg.port);
  }
  return std::string();
}

bool SbgcLink::open(const std::string & device, int baud, std::string * error)
{
  close();
  if (sbgc_open(&sb_, device.c_str(), baud) != 0) {
    last_error_ = sbgc_last_error(&sb_);
    if (error) {*error = last_error_;}
    return false;
  }
  sbgc_set_quiet(&sb_, 1);
  open_ = true;
  simulated_ = false;
  last_error_.clear();
  return true;
}

void SbgcLink::openSimulated()
{
  close();
  sbgc_open_simulated(&sb_);
  sbgc_set_quiet(&sb_, 1);
  open_ = true;
  simulated_ = true;
  last_error_.clear();
}

void SbgcLink::close()
{
  if (open_) {
    sbgc_close(&sb_);
    open_ = false;
  }
  simulated_ = false;
  have_realtime_ = false;
  realtime_fresh_ = false;
  have_board_info_ = false;
}

bool SbgcLink::send(uint8_t command, const uint8_t * payload, size_t len)
{
  if (!open_) {return false;}
  if (sbgc_send(&sb_, command, payload, len) != 0) {
    last_error_ = sbgc_last_error(&sb_);
    return false;
  }
  return true;
}

bool SbgcLink::sendControl(const WireControl & w)
{
  if (!open_) {return false;}
  if (sbgc_control_raw(&sb_, w.mode.data(), w.speed.data(), w.angle.data()) != 0) {
    last_error_ = sbgc_last_error(&sb_);
    return false;
  }
  return true;
}

bool SbgcLink::sendHome()
{
  if (!open_) {return false;}
  if (sbgc_home(&sb_) != 0) {
    last_error_ = sbgc_last_error(&sb_);
    return false;
  }
  return true;
}

bool SbgcLink::sendLevel()
{
  if (!open_) {return false;}
  if (sbgc_level(&sb_) != 0) {
    last_error_ = sbgc_last_error(&sb_);
    return false;
  }
  return true;
}

int SbgcLink::poll(int timeout_ms)
{
  if (!open_) {return -1;}
  const int n = sbgc_poll(&sb_, timeout_ms, &SbgcLink::frameThunk, this);
  if (n < 0) {last_error_ = sbgc_last_error(&sb_);}
  return n;
}

void SbgcLink::frameThunk(
  uint8_t command, const uint8_t * payload, size_t len, void * user)
{
  static_cast<SbgcLink *>(user)->onFrame(command, payload, len);
}

void SbgcLink::onFrame(uint8_t command, const uint8_t * payload, size_t len)
{
  ++frames_received_;

  switch (command) {
    case SBGC_CMD_REALTIME_DATA_3: {
        sbgc_realtime_t rt{};
        if (sbgc_parse_realtime_3(payload, len, &rt) == 0) {
          realtime_ = rt;
          have_realtime_ = true;
          realtime_fresh_ = true;
        } else {
          // A wrong-length payload is counted, never half-parsed into the
          // state a consumer will treat as the camera's real position.
          ++decode_errors_;
        }
        break;
      }
    case SBGC_CMD_BOARD_INFO:
    case SBGC_CMD_BOARD_INFO_3: {
        sbgc_board_info_t bi{};
        if (sbgc_parse_board_info(payload, len, &bi) == 0) {
          board_info_ = bi;
          have_board_info_ = true;
        } else {
          ++decode_errors_;
        }
        break;
      }
    default:
      // Everything else, CMD_CONFIRM and CMD_ERROR included, is counted as a
      // received frame and otherwise ignored. Acting on a frame this project
      // has not verified would be asserting meaning it cannot justify.
      break;
  }
}

bool SbgcLink::takeRealtimeFresh()
{
  const bool fresh = realtime_fresh_;
  realtime_fresh_ = false;
  return fresh;
}

}  // namespace sbgc_driver
