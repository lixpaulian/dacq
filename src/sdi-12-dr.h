/*
 * sdi-12-dr.h
 *
 * Copyright (c) 2017-2021, 2024 Lix N. Paulian (lix@paulian.net)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Created on: 13 Aug 2017 (LNP)
 */

#ifndef SDI_12_DR_H_
#define SDI_12_DR_H_

#include <cmsis-plus/rtos/os.h>
#include <dacq.h>
#include "uart-drv.h"

#ifndef SDI_BREAK_LEN
#define SDI_BREAK_LEN 20        // milliseconds
#endif

#ifndef MAX_CONCURRENT_REQUESTS
#define MAX_CONCURRENT_REQUESTS 10
#endif

#if defined (__cplusplus)

class sdi12_dr : public dacq
{
public:

  sdi12_dr (const char* name);

  ~sdi12_dr ();

  typedef enum
  {
    measure = 'M', //
    concurrent = 'C', //
    continuous = 'R', //
    verify = 'V',
    data = 'D'
  } method_t;

  typedef struct sdi12_
  {
    char addr;
    method_t method;
    uint8_t index;
    bool use_crc;
    int16_t max_waiting;
  } sdi12_t;

  void
  get_version (uint8_t& version_major, uint8_t& version_minor,
               uint8_t& version_patch) override;

  bool
  get_info (int id, char* info, size_t len) override;

  bool
  change_id (int id, int new_id) override;

  bool
  transparent (char* xfer_buff, int& len) override;

  bool
  retrieve (dacq_handle_t* dacqh) override;

  // --------------------------------------------------------------------

protected:

  // --------------------------------------------------------------------

private:

  int
  transaction (char* buff, size_t cmd_len, size_t len);

  bool
  start_measurement (sdi12_t* sdi, int& response_delay, uint8_t& measurements);

  bool
  wait_for_service_request (sdi12_t* sdi, int response_delay);

  bool
  get_data (sdi12_t* sdi, float* data, uint8_t* status, uint8_t& measurements);

  uint16_t
  calc_crc (uint16_t initial, uint8_t* buff, uint16_t buff_len);

  void
  force_break (void);

  void
  dump (const char* fmt, ...);

#if MAX_CONCURRENT_REQUESTS > 0
  bool
  retrieve_concurrent (dacq_handle_t* dacqh);

  static void*
  collect (void* args);

  typedef struct concurrent_msg_
  {
    dacq_handle_t dh;
    sdi12_t sdih;
    os::rtos::clock::timestamp_t response_delay;
  } concurent_msg_t;

  concurent_msg_t msgs_[MAX_CONCURRENT_REQUESTS];

  os::rtos::semaphore_counting sem_
    { "sdi12_dr", 2, 0 };
  os::rtos::thread th_
    { "sdi12-collect", collect, static_cast<void*> (this) };

#endif // MAX_CONCURRENT_REQUESTS > 0

  char last_sdi_addr_ = '?';
  os::rtos::clock::timestamp_t last_sdi_time_ = 0;
  os::rtos::clock::timestamp_t origin_;

  // transaction dump buffer, as the longest frame is 84 chars, it should be enough
  char dump_buffer_[128];

  // driver version
  static constexpr uint8_t VERSION_MAJOR = 1;
  static constexpr uint8_t VERSION_MINOR = 5;
  static constexpr uint8_t VERSION_PATCH = 4;

  // max 75 bytes values + 6 bytes address, CRC and CR/LF, word aligned
  static constexpr int longest_sdi12_frame = 84;

  // number of retries with break
  static constexpr int retries_with_break = 3;

  // timeout to wait on an already running SDI-12 transaction (in seconds)
  static constexpr uint32_t lock_timeout = (2 * 1000 * one_ms);

};

inline void
sdi12_dr::get_version (uint8_t& version_major, uint8_t& version_minor,
                       uint8_t& version_patch)
{
  version_major = VERSION_MAJOR;
  version_minor = VERSION_MINOR;
  version_patch = VERSION_PATCH;
}

inline void
sdi12_dr::force_break (void)
{
  last_sdi_time_ = 0;
}

// --------------------------------------------------------------------------

#endif /* (__cplusplus) */

#endif /* SDI_12_DR_H_ */
