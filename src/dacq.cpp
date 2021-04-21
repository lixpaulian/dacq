/*
 * dacq.cpp
 *
 * Copyright (c) 2017, 2018, 2020 Lix N. Paulian (lix@paulian.net)
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
#include <cmsis-plus/posix-io/file-descriptors-manager.h>

#include "dacq.h"

using namespace os;
using namespace os::rtos;

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
  dump_fn_ = nullptr;
}

/**
 * @brief Destructor.
 */
dacq::~dacq ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

/**
 * @brief Open a tty device and configure it accordingly. See termios.h for
 *      how the baudrate, c_size, parity are specified.
 * @param baudrate: the initial baud rate of the tty device.
 * @param c_size: number of bits to set on the tty.
 * @param parity: set the parity for the tty, if any.
 * @param rec_timeout: receive timeout, in ms.
 * @return true if successful, false otherwise.
 */
bool
dacq::open (speed_t baudrate, uint32_t c_size, uint32_t parity,
            uint32_t rec_timeout)
{
  bool result = false;
  dacq::err_num_t err_no;

  do
    {
      if (tty_)
        {
          err_no = tty_in_use;
          break;        // already in use
        }

      tty_ = static_cast<os::posix::tty*> (os::posix::open (name_, 0));
      if (tty_ == nullptr)
        {
          err_no = tty_open;
          break;
        }

      struct termios tio;
      if (tty_->tcgetattr (&tio) < 0)
        {
          dacq::close ();
          err_no = tty_attr;
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
          dacq::close ();
          err_no = tty_attr;
          break;
        }

      err_no = ok;
      result = true;
    }
  while (0);

  error = &err_[err_no];

  return result;
}

/**
 * @brief Provides a direct connection to the DACQ port.
 * @param fildes: a file descriptor of a stream that will directly
 *    communicate with the DACQ port.
 * @param timeout: timeout (in seconds) that will force a return if
 *    no characters are received for the specified period.
 */
void
dacq::direct (int fildes, int timeout)
{
  int count;
  uint8_t buff[512];

  console_ =
      static_cast<os::posix::tty*> (os::posix::file_descriptors_manager::io (
          fildes));

  // set a timeout on input
  struct termios tio_save;
  struct termios tio;
  console_->tcgetattr (&tio_save);
  console_->tcgetattr (&tio);
  tio.c_cc[VTIME] = 100;
  tio.c_cc[VMIN] = 0;
  console_->tcsetattr (TCSANOW, &tio);

  thread::attributes attr;
  attr.th_stack_size_bytes = 2048;

  thread th_dacq_rcv
    { "dacq-receive", dacq_rcv, static_cast<void*> (this), attr };

  int timeout_cnt = timeout / 10;

  while (timeout_cnt--)
    {
      while ((count = console_->read (buff, sizeof(buff))) > 0)
        {
          timeout_cnt = timeout / 20;
          if (count <= 3 && buff[0] == 0x18) // ctrl-X
            {
              count = -1;
              break;  // terminate direct command
            }
          if ((count = tty_->write (buff, count)) < 0)
            {
              break;
            }
        }
      if (count < 0)
        {
          break;        // tty error, exit
        }
    }

  // restore original termios
  console_->tcsetattr (TCSANOW, &tio_save);
}

/**
 * @brief Close the tty device.
 */
void
dacq::close (void)
{
  tty_->close ();
  tty_ = nullptr;
}

//----------------------------------------------------------------------

/**
 * @brief The thread handling the back channel of the direct connection.
 * @param args: pointer to the DACQ class.
 */
void*
dacq::dacq_rcv (void* args)
{
  int count;
  uint8_t buff[512];

  dacq* pdacq = (dacq*) args;

  do
    {
      if ((count = pdacq->tty_->read (buff, sizeof(buff))) > 0)
        {
          if (pdacq->console_->write (buff, count) < 0)
            {
              break;
            }
        }
    }
  while (count >= 0);

  return nullptr;
}
