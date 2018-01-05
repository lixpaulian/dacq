/*
 * dacq_serial.cpp
 *
 * Copyright (c) 2017, 2018 Lix N. Paulian (lix@paulian.net)
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

#include <inttypes.h>
#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/diag/trace.h>

#include <dacq.h>

using namespace os;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * @brief Constructor.
 * @param name: the path of a tty device (serial port).
 */
dacq::dacq (const char* name)
{
  trace::printf ("%s() %p\n", __func__, this);

  name_ = name;
  tty_ = nullptr;
}

/**
 * @brief Destructor.
 */
dacq::~dacq ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

/**
 * @brief Open a tty device and configure it accordingly.
 * @return true if successful, false otherwise.
 */
bool
dacq::open (speed_t baudrate, uint32_t c_size, uint32_t parity,
                   uint32_t rec_timeout)
{
  bool result = false;

  do
    {
      if (tty_)
        {
          error = &err_common[tty_in_use];
          break;        // already in use
        }

      tty_ = static_cast<os::posix::tty*> (os::posix::open (name_, 0));
      if (tty_ == nullptr)
        {
          error = &err_common[tty_open];
          break;
        }

      struct termios tio;
      if (tty_->tcgetattr (&tio) < 0)
        {
          error = &err_common[tty_attr];
          break;
        }
      tio.c_cc[VTIME] = (rec_timeout / 100) & 0xFF;
      tio.c_cc[VTIME_MS] = rec_timeout % 100;
      tio.c_cc[VMIN] = 0;

      // set baud rate, character size and parity, if any
      tio.c_ospeed = tio.c_ispeed = baudrate;
      tio.c_cflag = c_size | parity;

      if (tty_->tcsetattr (TCSANOW, &tio) < 0)
        {
          error = &err_common[tty_attr];
          break;
        }

      error = &err_common[ok];
      result = true;
    }
  while (0);

  return result;
}

/**
 * @brief Close the tty device.
 */
void
dacq::close (void)
{
  tty_->close ();
  tty_ = nullptr;
  error = &err_common[ok];
}

