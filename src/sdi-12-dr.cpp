/*
 * sdi-12-dr.cpp
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
 * Created on: 13 Aug 2017 (LNP)
 */

/*
 * This file implements the communication functionality of an SDI-12 data recorder.
 */

#include <inttypes.h>
#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/diag/trace.h>

#include "sdi-12-dr.h"

#ifndef SDI_BREAK_LEN
#define SDI_BREAK_LEN 20        // milliseconds
#endif

using namespace os;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

sdi12_dr::sdi12_dr (const char* name)
{
  trace::printf ("%s() %p\n", __func__, this);

  name_ = name;
  tty_ = nullptr;
}

sdi12_dr::~sdi12_dr ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

bool
sdi12_dr::open (void)
{
  bool result = false;

  do
    {
      if (tty_)
        {
          break;        // already in use
        }

      tty_ = static_cast<os::posix::tty*> (os::posix::open (name_, 0));
      if (tty_ == nullptr)
        {
          break;
        }

      struct termios tio;
      if (tty_->tcgetattr (&tio) < 0)
        {
          break;
        }
      tio.c_cc[VTIME] = 0;  // 50 ms timeout
      tio.c_cc[VTIME_MS] = 50;
      tio.c_cc[VMIN] = 0;

      // set baud rate to 1200
      tio.c_ospeed = tio.c_ispeed = 1200;

      if (tty_->tcsetattr (TCSANOW, &tio) < 0)
        {
          break;
        }
      result = true;
    }
  while (0);

  return result;
}

void
sdi12_dr::close (void)
{
  tty_->close ();
  tty_ = nullptr;
}

bool
sdi12_dr::ack_active (char addr)
{
  char buff[8];
  bool result = false;

  buff[0] = addr;
  buff[1] = '!';
  buff[2] = '\0';

  mutex_.lock ();
  if (transaction (buff, sizeof(buff)) == 3)
    {
      if (buff[0] == addr && buff[1] == '\r' && buff[2] == '\n')
        {
          result = true;
        }
    }
  mutex_.unlock ();

  return result;
}

bool
sdi12_dr::send_id (char addr, char* id, size_t id_len)
{
  bool result = false;

  if (id_len > 36)
    {
      id[0] = addr;
      id[1] = 'I';
      id[2] = '!';
      id[3] = '\0';

      mutex_.lock ();
      if (transaction (id, id_len) > 0)
        {
          if (id[0] == addr)
            {
              char *p;
              if ((p = strstr (id, "\r\n")) != nullptr)
                {
                  *p = '\0';    // replace cr with a null terminator
                  result = true;
                }
            }
        }
      mutex_.unlock ();
    }
  return result;
}

bool
sdi12_dr::change_address (char addr, char new_addr)
{
  char buff[8];
  bool result = false;

  buff[0] = addr;
  buff[1] = 'A';
  buff[2] = new_addr;
  buff[3] = '!';
  buff[4] = '\0';

  mutex_.lock ();
  if (transaction (buff, sizeof(buff)) == 3)
    {
      if (buff[0] == new_addr)
        {
          result = true;
        }
    }
  mutex_.unlock ();

  return result;
}

bool
sdi12_dr::sample_sensor (char addr, sdi12_dr::method_t method, uint8_t index,
                         bool use_crc, float* data, int& max_values)
{
  bool result = false;
  int waiting_time;
  int measurements;

  mutex_.lock ();

  do
    {
      if (method != sdi12_dr::continuous)
        {
          if (start_measurement (addr, method, index, use_crc, waiting_time,
                                 measurements) == false)
            {
              break;
            }
          if (wait_for_service_request (addr, waiting_time) == false)
            {
              break;
            }
          measurements = std::min (max_values, measurements);
          method = (method_t) 'D';
          index = 0;
        }
      if (send_data (addr, method, index, use_crc, data, measurements) == false)
        {
          break;
        }
      max_values = measurements;
      result = true;
    }
  while (0);

  mutex_.unlock ();

  return result;
}

bool
sdi12_dr::start_measurement (char addr, sdi12_dr::method_t method,
                             uint8_t index, bool use_crc, int& response_delay,
                             int& measurements)
{
  bool result = false;
  char buff[32];
  int count;

  if (index < 10)
    {
      buff[0] = addr;
      buff[1] = method;
      buff[2] = use_crc ? 'C' : index ? index + '0' : '!';
      buff[3] = use_crc ? (index ? index + '0' : '!') : '\0';
      buff[4] = (use_crc && index) ? '!' : '\0';
      buff[5] = '\0';

      count = transaction (buff, sizeof(buff));
      if (count > 6) // we need at least the delay and the number of meas'ments
        {
          buff[count - 2] = '\0';
          measurements = atoi (&buff[4]);
          buff[4] = '\0';
          response_delay = atoi (&buff[1]);
          result = true;
        }
    }
  return result;
}

bool
sdi12_dr::wait_for_service_request (char addr, int response_delay)
{
  bool result = false;
  char buff[4];
  size_t res;
  struct termios tio;

  if (tty_->tcgetattr (&tio) >= 0)
    {
      // save original values
      cc_t vtime = tio.c_cc[VTIME];
      cc_t vtime_ms = tio.c_cc[VTIME_MS];

      tio.c_cc[VTIME] = 10; // wait for one second timeout
      tio.c_cc[VTIME_MS] = 0;

      if (tty_->tcsetattr (TCSANOW, &tio) >= 0)
        {
          do
            {
              res = tty_->read (buff, sizeof(buff));
            }
          while (--response_delay > 0 && res == 0);
          if (res > 0 && addr == buff[0])
            {
              last_sdi_time_ = os::rtos::sysclock.now ();
              last_sdi_addr_ = addr;
            }
          result = true;
        }

      tio.c_cc[VTIME] = vtime; // restore original timeout values
      tio.c_cc[VTIME_MS] = vtime_ms;
      if (tty_->tcsetattr (TCSANOW, &tio) < 0)
        {
          result = false;
        }
    }
  return result;
}

bool
sdi12_dr::send_data (char addr, method_t method, uint8_t index, bool use_crc,
                     float* data, int& measurements)
{
  bool result = false;
  char buff[SDI12_LONGEST_FRAME];
  char request = index + '0';
  int parsed = 0, count;

  do
    {
      buff[0] = addr;
      buff[1] = method;
      buff[2] = request;
      buff[3] = '!';
      buff[4] = '\0';
      count = transaction (buff, sizeof(buff));

      if (count > 0 && addr == buff[0])
        {
          if (use_crc)
            {
              char* p = buff + count - 5;  // p points on the first CRC byte
              uint16_t incoming_crc = (*p++ & 0x3F) << 12;
              incoming_crc += (*p++ & 0x3F) << 6;
              incoming_crc += (*p & 0x3F);
              if (incoming_crc
                  != sdi12_dr::calc_crc (0, (uint8_t*) buff, count - 5))
                {
                  break;
                }
            }

          char *p, *r = buff + 1; // skip address
          buff[count - (use_crc ? 5 : 2)] = '\0'; // terminate string
          do
            {
              p = r;
              data[parsed] = strtof (p, &r);
              if (data[parsed] == 0 && p == r)
                {
                  break;    // no conversion possible, exit
                }
              parsed++;
            }
          while (*r != '\0' && parsed < measurements);

          if (method == sdi12_dr::continuous)
            {
              break;
            }
        }
    }
  while (request++ < '9' && count > 0 && parsed < measurements);

  if (parsed)
    {
      measurements = parsed;
      result = true;
    }

  return result;
}

// -------------------------------------------------------------------------

int
sdi12_dr::transaction (char* buff, size_t buff_len)
{
  int result = 0;
  int retries_with_break = 3;
  char response[SDI12_LONGEST_FRAME];

  do
    {
      // check if we need to send a break
      if (last_sdi_addr_ != buff[0]
          || (os::rtos::sysclock.now () - last_sdi_time_) > 80)
        {
          // send a break at least 12 ms long
          tty_->tcsendbreak (SDI_BREAK_LEN);
        }
      last_sdi_addr_ = buff[0];     // replace last address

      // wait at least 8.33 ms
      rtos::sysclock.sleep_for (10);

      int retries = 3;
      bool valid_sdi12 = false;
      do
        {
          // send request
          if ((result = tty_->write (buff, strlen (buff))) < 0)
            {
              break;
            }
          // wait for the end of transmission
          rtos::sysclock.sleep_for ((83 * strlen (buff)) / 10);
          last_sdi_time_ = os::rtos::sysclock.now ();

          // read response, if any
          result = tty_->read (response, sizeof(response));

          // check if the frame is valid
          if (result > 0 && response[result - 1] == '\n'
              && response[result - 2] == '\r')
            {
              valid_sdi12 = true;
            }
        }
      while (valid_sdi12 == false && --retries);

      if (valid_sdi12)
        {
          strncpy (buff, response, buff_len);
          last_sdi_time_ = os::rtos::sysclock.now ();
          break;
        }
      else
        {
          // wait a little before a break and new triplet sequence
          rtos::sysclock.sleep_for (10);

          // force a break, even if the timeout did not expire
          last_sdi_time_ = 0;
        }
    }
  while (--retries_with_break);

  return result;
}

uint16_t
sdi12_dr::calc_crc (uint16_t initial, uint8_t* buff, uint16_t buff_len)
{

  for (uint16_t count = 0; count < buff_len; count++)
    {
      initial ^= *buff++;
      int cnt_byte = 8;
      while (cnt_byte-- > 0)
        {
          if (initial & 1)
            {
              initial >>= 1;
              initial ^= 0xA001;
            }
          else
            initial >>= 1;
        }
    }
  return initial;
}

#pragma GCC diagnostic pop
