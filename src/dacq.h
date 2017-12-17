/*
 * dacq.h
 *
 * Copyright (c) 2017 Lix N. Paulian (lix@paulian.net)
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
 * Created on: 19 Nov 2017 (LNP)
 */

#ifndef DACQ_H_
#define DACQ_H_

#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/posix/termios.h>

#include "uart-drv.h"

#if defined (__cplusplus)

using namespace os;
using namespace os::rtos;

class dacq
{
public:

  dacq (const char* name);

  virtual
  ~dacq ();

  typedef struct dacq_handle_
  {
    time_t date;        // date/time stamp for this data set
    float* data;        // pointer on an array of tags
    uint8_t* status;    // pointer on an array of tag statuses
    uint8_t data_count; // number of values (tags)
    void* impl;        // pointer (normally to a struct) implementation specific
    void* cb_process;   // pointer on a user object that handles the values
    bool
    (*cb) (struct dacq_handle_*);
  } dacq_handle_t;

  bool
  open (speed_t baudrate, uint32_t c_size, uint32_t parity,
        uint32_t rec_timeout);

  void
  close (void);

  virtual void
  get_version (uint8_t& version_major, uint8_t& version_minor) = 0;

  virtual int
  transaction (char* buff, size_t cmd_len, size_t len) = 0;

  virtual bool
  get_info (int id, char* ver, size_t len) = 0;

  virtual bool
  retrieve (dacq_handle_t* dacqh) = 0;

  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_BIT_MISSING = 1;
  static constexpr uint8_t STATUS_BIT_IMPLAUSIBILE = 2;

protected:

  posix::tty* tty_;
  mutex mutex_
    { "dacq" };

private:
  const char* name_;

};

#endif /* (__cplusplus) */

#endif /* DACQ_H_ */