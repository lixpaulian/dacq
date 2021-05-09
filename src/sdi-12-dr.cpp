/*
 * sdi-12-dr.cpp
 *
 * Copyright (c) 2017-2021 Lix N. Paulian (lix@paulian.net)
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
 * This file implements the communication functionality of an SDI-12
 * data recorder.
 */

#include <inttypes.h>
#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/diag/trace.h>

#include "sdi-12-dr.h"

#define SDI_DEBUG false

using namespace os;
using namespace os::rtos;

/**
 * @brief Constructor.
 * @param name: the path of an RS-485 tty device (serial port).
 */
sdi12_dr::sdi12_dr (const char* name) :
    dacq
      { name }
{
  trace::printf ("%s() %p\n", __func__, this);
}

/**
 * @brief Destructor.
 */
sdi12_dr::~sdi12_dr ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

/**
 * @brief Implementation of the "Send ID" command (sensor information).
 * @param id: sensor's address.
 * @param info: buffer where the sensor identification string will be returned.
 * @param len: length of the buffer; if the ID string is longer than the buffer,
 *      it will be truncated.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::get_info (int id, char* info, size_t len)
{
  bool result = false;
  int retries = retries_with_break;

  if (len > 36)
    {
      if (mutex_.timed_lock (lock_timeout) == result::ok)
        {
          origin_ = sysclock.now ();
          do
            {
              info[0] = id;
              info[1] = 'I';
              info[2] = '!';

              if (transaction (info, 3, len) > 0)
                {
                  if (info[0] != id)
                    {
                      error = &err_[unexpected_answer];
                    }
                  else
                    {
                      char* p;
                      if ((p = strstr (info, "\r\n")) != nullptr)
                        {
                          *p = '\0';    // replace cr with a null terminator
                          memmove (info, info + 1, strlen (info)); // remove address
                          result = true;
                          break;
                        }
                    }
                }
              force_break ();
            }
          while (--retries);
          mutex_.unlock ();
        }
      else
        {
          error = &err_[dacq_busy];
        }
    }
  else
    {
      error = &err_[buffer_too_small];
    }

  if (result == false)
    {
      *info = '\0';
    }

  return result;
}

/**
 * @brief Change sensor address (id).
 * @param id: sensor address to change.
 * @param new_id: new address.
 * @return true if the address change was successful, false otherwise.
 */
bool
sdi12_dr::change_id (int id, int new_id)
{
  bool result = false;
  char buffer[8];
  int retries = retries_with_break;

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      origin_ = sysclock.now ();
      do
        {
          buffer[0] = id;
          buffer[1] = 'A';
          buffer[2] = new_id;
          buffer[3] = '!';

          if (transaction (buffer, 4, sizeof(buffer)) > 0)
            {
              if (buffer[0] == new_id)
                {
                  result = true;
                  break;
                }
              else
                {
                  error = &err_[unexpected_answer];
                }
            }
          force_break ();
        }
      while (--retries);
      mutex_.unlock ();
    }
  else
    {
      error = &err_[dacq_busy];
    }

  return result;
}

/**
 * @brief Execute a transparent command. This function may be used also for the
 *      extended commands "X".
 * @param xfer_buff: buffer with command to send (null terminated) and receive
 *      the answer to/from the sensor.
 * @param len: buffer length; on return it contains the length of the answer.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::transparent (char* xfer_buff, int& len)
{
  bool result = false;
  int retries = retries_with_break;
  char buff[longest_sdi12_frame];
  size_t in_len = std::min (len, longest_sdi12_frame);

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      memcpy (buff, xfer_buff, in_len);
      origin_ = sysclock.now ();
      do
        {
          if ((len = transaction (xfer_buff, strlen (xfer_buff), len)) > 0)
            {
              xfer_buff[len] = '\0';
              result = true;
              break;
            }
          else
            {
              memcpy (xfer_buff, buff, in_len);
            }
          force_break ();
        }
      while (--retries);
      mutex_.unlock ();
    }
  else
    {
      error = &err_[dacq_busy];
    }

  return result;
}

/**
 * @brief Retrieve data.
 * @param dacqh: pointer on a structure of type dacq_handle_t containing all
 *      sensor relevant data.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::retrieve (dacq_handle_t* dacqh)
{
  bool result = false;
  int waiting_time;
  uint8_t measurements = 0;
  sdi12_t* sdi = (sdi12_t*) dacqh->impl;

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      do
        {
          // set default for all status bits to "missing"
          memset (dacqh->status, STATUS_BIT_MISSING, dacqh->data_count);
          origin_ = sysclock.now ();

          if (sdi->method != sdi12_dr::continuous)
            {
#if MAX_CONCURRENT_REQUESTS > 0
              if (sdi->method == sdi12_dr::concurrent)
                {
                  // we initiate a real concurrent retrieve (SDI-12 command C)
                  if (retrieve_concurrent (dacqh) == false)
                    {
                      break;
                    }
                }
              else
#endif
                {
                  // initiate a sequential retrieve
                  if (start_measurement (sdi, waiting_time, measurements)
                      == false)
                    {
                      break;
                    }
                  // wait for the sensor to send a service request
                  if (wait_for_service_request (sdi, waiting_time) == false)
                    {
                      break;
                    }
                }
            }

#if MAX_CONCURRENT_REQUESTS > 0
          if (sdi->method != sdi12_dr::concurrent)
#endif
            {
              measurements = std::min (dacqh->data_count, measurements);
              if (measurements || sdi->method == sdi12_dr::continuous)
                {
                  if (sdi->method != sdi12_dr::continuous)
                    {
                      sdi->method = sdi12_dr::data;
                      sdi->index = 0;
                    }

                  // get sensor data
                  if (get_data (sdi, dacqh->data, dacqh->status, measurements)
                      == false)
                    {
                      break;
                    }
                  error = &err_[ok];
                  result = true;
                }
              else
                {
                  error = &err_[no_sensor_data];
                }
            }
        }
      while (0);

#if MAX_CONCURRENT_REQUESTS > 0
      if (sdi->method != sdi12_dr::concurrent)
#endif
        {
          dacqh->data_count = measurements;
          if (dacqh->cb != nullptr)
            {
              dacqh->cb (dacqh);
            }
        }
      mutex_.unlock ();
    }
  else
    {
      error = &err_[dacq_busy];
    }

  return result;
}

// --------------------------------------------------------------------------

// --------------------------------------------------------------------------

/**
 * @brief Perform an SDI-12 transaction using the RS-485 tty.
 * @param buff: buffer containing the SDI-12 request; on return, the buffer
 *      should contain the SDI-12 answer from the sensor.
 * @param cmd_len: command length.
 * @param len: buffer's total length.
 * @return: Number of characters returned, or -1 on error.
 */
int
sdi12_dr::transaction (char* buff, size_t cmd_len, size_t len)
{
  int result = 0;
  err_num_t err_no = timeout;
  char answer[longest_sdi12_frame];
  int first;

  // check if we need to send a break
  if (last_sdi_addr_ != buff[0] || (sysclock.now () - last_sdi_time_) > 85)
    {
      // send a break at least 12 ms long
      first = sysclock.now () - origin_;
      tty_->tcsendbreak (SDI_BREAK_LEN);
      dump ("%05d-%05d --> break", first, first + SDI_BREAK_LEN);
#if SDI_DEBUG == true
          trace::printf ("%s(): break\n", __func__);
#endif
    }
  last_sdi_addr_ = buff[0];         // replace last address

  // wait at least 8.33 ms
  sysclock.sleep_for (10);

  int retries = 3;
  tty_->tcflush (TCIOFLUSH);        // clear input
  do
    {
#if SDI_DEBUG == true
          trace::printf ("%s(): sent %.*s\n", __func__, cmd_len, buff);
#endif
      // compute time taken by write
      os::rtos::clock::timestamp_t xmit_end = sysclock.now ()
          + (83 * cmd_len) / 10;

      // send request
      first = sysclock.now () - origin_;
      dump ("%05d-%05d --> %.*s", first, first + ((cmd_len * 8333) / 1000),
            cmd_len, buff);
      if ((result = tty_->write (buff, cmd_len)) < 0)
        {
          dump ("%05d-~~~~~ --> write failed", sysclock.now () - origin_);
          err_no = tty_error;
          break;
        }

      // wait for the end of transmission
      sysclock.sleep_until (xmit_end);
      last_sdi_time_ = sysclock.now ();

      // read response, if any
      size_t offset = 0;
      bool valid_sdi12 = false;
      do
        {
          result = tty_->read (answer + offset, sizeof(answer) - offset);
          if (result <= 0)
            {
              break;
            }
          offset += result;
          // check if the frame is valid
          if (answer[offset - 1] == '\n' && answer[offset - 2] == '\r')
            {
              valid_sdi12 = true;
              result = offset;
              break;
            }
          // we read until we get a valid frame or overflow the buffer
        }
      while (offset < sizeof(answer));

      if (valid_sdi12 && result > 0)
        {
#if SDI_DEBUG == true
              trace::printf ("%s(): received %.*s\n", __func__, result, answer);
#endif
          os::rtos::clock::timestamp_t wait_end = sysclock.now () + 20;
          first = sysclock.now () - origin_ - (((result + 1) * 8333) / 1000);
          int last = sysclock.now () - origin_ - 8;
          dump ("%05d-%05d <-- %.*s", first, last, result, answer);
          sysclock.sleep_until (wait_end);
          strncpy (buff, answer, len);
          last_sdi_time_ = sysclock.now ();
          err_no = ok;
          break;
        }
      else
        {
#if SDI_DEBUG == true
              trace::printf ("%s(): timeout\n", __func__);
#endif
          dump ("~~~~~-%05d <-- timeout", sysclock.now () - origin_);
        }
    }
  while (--retries);

  error = &err_[err_no];

  return result;
}

/**
 * @brief Start a two-steps measurement using "M", "C" or "V" SDI-12 commands.
 * @param sdi: a asdi12_t type structure defining a sensor.
 * @param response_delay: number of seconds the sensor needs to return the values.
 * @param measurements: the number of values that will be returned by the sensor.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::start_measurement (sdi12_t* sdi, int& response_delay,
                             uint8_t& measurements)
{
  bool result = false;
  char buff[32];
  int count;
  int retries = retries_with_break;

  if (sdi->index < 10)
    {
      do
        {
          buff[0] = sdi->addr;
          buff[1] = sdi->method;
          buff[2] = sdi->use_crc ? 'C' : sdi->index ? sdi->index + '0' : '!';
          buff[3] =
              sdi->use_crc ?
                  (sdi->index ? sdi->index + '0' : '!') :
                  (sdi->index ? '!' : '\0');
          buff[4] = (sdi->use_crc && sdi->index) ? '!' : '\0';
          buff[5] = '\0';

          if ((count = transaction (buff, strlen (buff), sizeof(buff))) > 0)
            {
              if (sdi->addr != buff[0] || count < 7)
                {
                  // answer from wrong sensor or too short
                  error = &err_[unexpected_answer];
                }
              else
                {
                  buff[count - 2] = '\0';
                  measurements = (uint8_t) atoi (&buff[4]);
                  buff[4] = '\0';
                  response_delay = atoi (&buff[1]);
                  result = true;
                  break;
                }
            }
          force_break ();
        }
      while (--retries);
    }
  else
    {
      error = &err_[invalid_index];
    }

  return result;
}

/**
 * @brief Wait for a service request from a sensor.
 * @param sdi: a asdi12_t type structure defining a sensor.
 * @param response_delay: number of seconds to wait until the sensor answer,
 *      after which the function returns.
 * @return true if successful, false otherwise.
 * @note The function returns true either if the sensor sent a service request,
 *      or if the timeout expired.
 */
bool
sdi12_dr::wait_for_service_request (sdi12_t* sdi, int response_delay)
{
  bool result = false;
  err_num_t err_no = tty_attr;
  char buff[4];
  size_t res;
  struct termios tio;

  if (sdi->method == sdi12_dr::concurrent)
    {
      sysclock.sleep_for (response_delay * 1000);
      err_no = ok;
      result = true;
    }
  else if (tty_->tcgetattr (&tio) >= 0)
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

          if (res > 0 && sdi->addr == buff[0])
            {
              // got a service request
              last_sdi_time_ = sysclock.now ();
              last_sdi_addr_ = sdi->addr;
              int first = sysclock.now () - origin_
                  - (((res + 1) * 8333) / 1000);
              int last = sysclock.now () - origin_ - 8;
              dump ("%05d-%05d <-- %.*s", first, last, res, buff);
            }
          else
            {
              // timeout is up, add half a second wait before requesting the data,
              // for sensors with flawed implementations
              sysclock.sleep_for (500);
            }

#if SDI_DEBUG == true
          if (res > 0)
            {
              trace::printf ("%s(): received %.*s\n", __func__, res, buff);
            }
          else
            {
              trace::printf ("%s(): timeout\n", __func__);
            }
#endif

          tio.c_cc[VTIME] = vtime; // restore original timeout values
          tio.c_cc[VTIME_MS] = vtime_ms;
          if (tty_->tcsetattr (TCSANOW, &tio) == 0)
            {
              err_no = ok;
              result = true;
            }
        }
    }
  error = &err_[err_no];

  return result;
}

/**
 * @brief Implementation of the SDI-12 "Send Data" command.
 * @param sdi: a asdi12_t type structure defining a sensor.
 * @param data: pointer on an array of floats where the data will be returned.
 * @param status: pointer on an array of sensor statuses.
 * @param measurements: maximum number of values allowed in 'data'. On return,
 *      it contains the actual number of values returned by the sensor.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::get_data (sdi12_t* sdi, float* data, uint8_t* status,
                    uint8_t& measurements)
{
  bool result = false;
  char buff[longest_sdi12_frame];
  char request = sdi->index + '0';
  uint8_t parsed = 0;
  int count;

  if (data != nullptr && status != nullptr)
    {
      // set all status bytes to "missing values"
      memset (status, STATUS_BIT_MISSING, measurements);
      do
        {
          int retries = retries_with_break;

          do
            {
              buff[0] = sdi->addr;
              buff[1] = sdi->method;
              buff[2] =
                  (sdi->method == sdi12_dr::continuous && sdi->use_crc) ?
                      'C' : request;
              buff[3] =
                  (sdi->method == sdi12_dr::continuous && sdi->use_crc) ?
                      request : '!';
              buff[4] =
                  (sdi->method == sdi12_dr::continuous && sdi->use_crc) ?
                      '!' : '\0';
              buff[5] = '\0';

              if ((count = transaction (buff, strlen (buff), sizeof(buff))) > 0)
                {
                  do
                    {
                      if (sdi->addr != buff[0] || (sdi->use_crc && count < 6))
                        {
                          error = &err_[unexpected_answer];
                          break;
                        }
                      if (sdi->use_crc)
                        {
                          // verify the CRC
                          char* p = buff + count - 5; // p points on the first CRC byte
                          uint16_t incoming_crc = (*p++ & 0x3F) << 12;
                          incoming_crc += (*p++ & 0x3F) << 6;
                          incoming_crc += (*p & 0x3F);
                          if (incoming_crc
                              != sdi12_dr::calc_crc (0, (uint8_t*) buff,
                                                     count - 5))
                            {
                              error = &err_[crc_error];
                              break;
                            }
                        }
                      char* p, * r = buff + 1; // skip address
                      buff[count - (sdi->use_crc ? 5 : 2)] = '\0'; // terminate string
                      do
                        {
                          p = r;
                          data[parsed] = strtof (p, &r);
                          if (data[parsed] == 0 && p == r)
                            {
                              error = &err_[conversion_to_float_error];
                              break;    // conversion to float error, exit
                            }
                          status[parsed++] = STATUS_OK;
                        }
                      while (*r != '\0');
                    }
                  while (0);
                }
              if (error->error_number == ok)
                {
                  break;
                }
              else
                {
                  force_break ();
                }
            }
          while (--retries);

          if (sdi->method == sdi12_dr::continuous)
            {
              break;
            }
        }
      while (request++ < '9' && count > 0 && parsed < measurements
          && error->error_number == ok);

      // any values retrieved?
      if (parsed)
        {
          measurements = parsed;
          result = true;
        }
    }

  return result;
}

/**
 * @brief Compute the CRC of an SDI-12 string.
 * @param initial: initial CRC value (normally 0).
 * @param buff: buffer containing the SDI-12 string.
 * @param buff_len: length of the SDI-12 string.
 * @return Computed CRC value for the SDI-12 string.
 */
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

/**
 * @brief Dump the transaction dialogue to a hooked-up function (for protocol debug).
 * @param fmt: formatted string (printf() style).
 */
void
sdi12_dr::dump (const char* fmt, ...)
{
  if (dump_fn_)
    {
      va_list ap;

      memset (dump_buffer_, 0, sizeof(dump_buffer_));
      va_start(ap, fmt);
      vsnprintf (dump_buffer_, sizeof(dump_buffer_), fmt, ap);
      va_end(ap);

      dump_fn_ (dump_buffer_);
    }
}

#if MAX_CONCURRENT_REQUESTS > 0
/**
 * @brief Sample asynchronously a sensor; this function does not block. After the
 *      data is collected, a user call-back function will be called.
 * @param dacqh: pointer on a structure of type dacq_handle_t containing all
 *      sensor relevant data.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::retrieve_concurrent (dacq_handle_t* dacqh)
{
  bool result = false;
  int waiting_time;
  uint8_t measurements;
  concurent_msg_t* pmsg = nullptr;
  sdi12_t* sdi = (sdi12_t*) dacqh->impl;

  for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
      if (msgs_[i].sdih.addr == static_cast<sdi12_t*> (dacqh->impl)->addr)
        {
          // this sensor is already in a transaction, abort
          error = &err_[sensor_busy];
          return result;
        }
    }

  // search for a free entry in the table
  for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
      if (msgs_[i].sdih.addr == 0)
        {
          pmsg = &msgs_[i];
          break;    // found
        }
    }

  if (pmsg)
    {
      // initiate a concurrent measurement
      if (start_measurement (sdi, waiting_time, measurements) == true)
        {
          // copy sensor data to the table
          memcpy (&pmsg->dh, dacqh, sizeof(dacq_handle_t));
          memcpy (&pmsg->sdih, sdi, sizeof(sdi12_t));

          // update the entry with ETA and number of expected values
          pmsg->response_delay = sysclock.now () + waiting_time * 1000;
          pmsg->dh.data_count = std::min (dacqh->data_count, measurements);

          // inform the collect task that a new entry is available
          if (sem_.post () == result::ok)
            {
              result = true;
            }
        }
    }
  else
    {
      error = &err_[too_many_requests];
    }

  return result;
}

/**
 * @brief Thread to handle asynchronous sensor data sampling.
 * @param args: pointer on the class ("this").
 */
void*
sdi12_dr::collect (void* args)
{
  sdi12_dr* self = static_cast<sdi12_dr*> (args);
  semaphore_counting* sem = &self->sem_;
  concurent_msg_t* pmsg = nullptr;
  result_t result;
  clock::duration_t timeout = 0xFFFFFFFF; // forever

  memset (self->msgs_, 0, MAX_CONCURRENT_REQUESTS * sizeof(concurent_msg_t));

  while (true)
    {
      result = sem->timed_wait (timeout);
      if (result != result::ok)
        {
          // if timeout, the first sensor in line is now ready
          if (pmsg != nullptr)
            {
              self->mutex_.lock ();

              // get sensor data
              pmsg->sdih.method = (method_t) 'D';
              if (self->get_data (&pmsg->sdih, pmsg->dh.data, pmsg->dh.status,
                                  pmsg->dh.data_count) == true)
                {
                  pmsg->dh.impl = &pmsg->sdih;  // update sensor handle
                  if (pmsg->dh.cb != nullptr)
                    {
                      pmsg->dh.cb (&pmsg->dh);  // user callback
                    }
                }
              pmsg->sdih.addr = 0;
              pmsg = nullptr;       // all done here, clear entry
              self->mutex_.unlock ();
            }
        }
      // otherwise we might have a new entry from the main thread
      // search for the next sensor (if any) ready to deliver its data
      clock::duration_t nearest_request = 0xFFFFFFFF;
      pmsg = nullptr;
      for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
        {
          if (self->msgs_[i].sdih.addr != 0)
            {
              if (self->msgs_[i].response_delay < nearest_request)
                {
                  nearest_request = self->msgs_[i].response_delay;
                  pmsg = &self->msgs_[i];
                }
            }
        }

      // compute the timeout for the next wake-up
      timeout = 0xFFFFFFFF;
      if (pmsg != nullptr)
        {
          timeout =
              pmsg->response_delay > sysclock.now () ?
                  pmsg->response_delay - sysclock.now () : 0;
        }
    }

  return nullptr;
}
#endif // MAX_CONCURRENT_REQUESTS > 0
