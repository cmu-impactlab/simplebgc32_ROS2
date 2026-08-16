// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// The one thing in this package that owns the serial port.
//
// sbgc_t has no internal locking, and the upstream project keeps it in a
// single thread for that reason. Nothing here adds a mutex; the contract is
// that one owner drives it. The node satisfies that by putting every callback
// that touches this object in one mutually-exclusive callback group.

#ifndef SBGC_DRIVER__SBGC_LINK_HPP_
#define SBGC_DRIVER__SBGC_LINK_HPP_

#include <cstdint>
#include <string>

#include "sbgc_driver/gimbal_core.hpp"
#include "sbgc_params.h"  // NOLINT(build/include_subdir)

namespace sbgc_driver
{

class SbgcLink
{
public:
  SbgcLink();
  ~SbgcLink();

  SbgcLink(const SbgcLink &) = delete;
  SbgcLink & operator=(const SbgcLink &) = delete;

  // Returns false and fills `error` rather than throwing, because a missing
  // port is an ordinary runtime condition on a robot, not a program fault.
  bool open(const std::string & device, int baud, std::string * error);
  void openSimulated();
  void close();

  bool isOpen() const {return open_;}
  bool simulated() const {return simulated_;}

  // Resolve an empty request against the vendor GUI's saved settings and
  // /dev/serial/by-id. Returns an empty string when nothing was found.
  static std::string discoverPort(const std::string & requested);

  bool send(uint8_t command, const uint8_t * payload = nullptr, size_t len = 0);
  bool sendControl(const WireControl & w);

  // Home and level are the vendor's own auto-task frames, reproduced
  // byte-for-byte by the protocol library. They are separate calls rather than
  // a WireControl because the mode bytes carry the AUTO_TASK flag, which is
  // not something GimbalCore's ordinary output should ever be able to set.
  bool sendHome();
  bool sendLevel();

  // Read and dispatch whatever has arrived. Returns the number of frames
  // consumed, or -1 on a read error, which the caller should treat as a lost
  // link rather than retry blindly.
  int poll(int timeout_ms);

  bool haveRealtime() const {return have_realtime_;}
  const sbgc_realtime_t & realtime() const {return realtime_;}

  bool haveBoardInfo() const {return have_board_info_;}
  const sbgc_board_info_t & boardInfo() const {return board_info_;}

  uint32_t framesReceived() const {return frames_received_;}
  uint32_t decodeErrors() const {return decode_errors_;}

  // True when a realtime frame arrived since the last call, so the caller can
  // stamp it with its own clock rather than this file guessing at one.
  bool takeRealtimeFresh();

  const std::string & lastError() const {return last_error_;}

private:
  static void frameThunk(
    uint8_t command, const uint8_t * payload, size_t len, void * user);
  void onFrame(uint8_t command, const uint8_t * payload, size_t len);

  sbgc_t sb_{};
  bool open_ = false;
  bool simulated_ = false;

  sbgc_realtime_t realtime_{};
  bool have_realtime_ = false;
  bool realtime_fresh_ = false;

  sbgc_board_info_t board_info_{};
  bool have_board_info_ = false;

  uint32_t frames_received_ = 0;
  uint32_t decode_errors_ = 0;

  std::string last_error_;
};

}  // namespace sbgc_driver

#endif  // SBGC_DRIVER__SBGC_LINK_HPP_
