/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EMANETYPES_HEADER_
#define EMANETYPES_HEADER_

#include <cstdint>
#include <chrono>
#include <string>
#include <list>

#ifdef OFFLINE_TEST_TIMER
namespace detail
{
struct offline_test_clock
{
  using duration = std::chrono::system_clock::duration;//= std::chrono::microseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<offline_test_clock>;
  static const bool is_steady = true;

  static time_point now() noexcept
  {
    return now_;
  }

  static void set_now(time_point t)
  {
    now_ = t;
  }

  static void add_time(duration d)
  {
    now_ += d;
  }

  //template<typename T>
  //static std::time_t to_time_t(const T& t) noexcept
  static std::time_t to_time_t(time_point& t) noexcept
  {
    return std::time_t {};
    //return std::chrono::system_clock::to_time_t(
     // std::chrono::time_point_cast<std::chrono::system_clock::duration>(t));
      //std::chrono::time_point_cast<std::chrono::steady_clock::duration>(t));
      //std::chrono::seconds{0}});
    //t.time_since_epoch());
      //std::chrono::steady_clock::time_point{t.time_since_epoch()});
      //std::chrono::time_point_cast<std::chrono::system_clock::duration>(t));
  }

private:
  static time_point now_;
};
}
#endif

namespace EMANE
{
using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;
using Microseconds = std::chrono::microseconds;
using Nanoseconds = std::chrono::nanoseconds;
using DoubleSeconds = std::chrono::duration<double>;
#ifdef OFFLINE_TEST_TIMER
using Clock = detail::offline_test_clock;
#else
using Clock = std::chrono::system_clock;
#endif
using Duration = Clock::duration;
using TimePoint = Clock::time_point;

using NEMId = std::uint16_t;
using EventId = std::uint16_t;
using TimerEventId = std::size_t;

using PlatformId = std::uint16_t;
using AntennaProfileId = std::uint16_t;
using ControlMessageId = std::uint16_t;
using RegistrationId = std::uint16_t;
using BuildId = std::uint32_t;

using LengthPrefix = std::uint16_t;

using Priority = std::uint8_t;

using Strings = std::list<std::string>;

// All 1's NEMId represents a broad cast packet
constexpr NEMId NEM_BROADCAST_MAC_ADDRESS{std::numeric_limits<NEMId>::max()};
} // namespace EMANE

#endif // EMANETYPES_HEADER_
