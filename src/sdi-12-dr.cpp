/*
 * sdi-12-dr.cpp
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

  if (len > 36)
    {
      info[0] = id;
      info[1] = 'I';
      info[2] = '!';

      if (mutex_.timed_lock (lock_timeout) == result::ok)
        {
          if (transaction (info, 3, len) > 0)
            {
              if (info[0] == id)
                {
                  char *p;
                  if ((p = strstr (info, "\r\n")) != nullptr)
                    {
                      *p = '\0';    // replace cr with a null terminator
                      memmove (info, info + 1, strlen (info)); // remove address
                      result = true;
                    }
                }
            }
          mutex_.unlock ();
        }
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

  buffer[0] = id;
  buffer[1] = 'A';
  buffer[2] = new_id;
  buffer[3] = '!';

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      if (transaction (buffer, 4, sizeof(buffer)) > 0)
        {
          if (buffer[0] == new_id)
            {
              result = true;
            }
        }
      mutex_.unlock ();
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

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      if ((len = transaction (xfer_buff, strlen (xfer_buff), len)) > 0)
        {
          result = true;
        }
      mutex_.unlock ();
    }

  return result;
}

/**
 * @brief Retrieve data; this function blocks until the sensor returns the data.
 * @param dacqh: pointer on a structure of type dacq_handle_t containing all
 *      sensor relevant data.
 * @return true if successful, false otherwise.
 */
bool
sdi12_dr::retrieve (dacq_handle_t* dacqh)
{
  bool result = false;
  int waiting_time;
  uint8_t measurements;
  sdi12_t* sdi = (sdi12_t*) dacqh->impl;

  if (mutex_.timed_lock (lock_timeout) == result::ok)
    {
      do
        {
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
                  if (wait_for_service_request (sdi->addr, waiting_time)
                      == false)
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
              sdi->method = (method_t) 'D';
              sdi->index = 0;

              // get data from sensor
              if (get_data (sdi, dacqh->data, dacqh->status, measurements)
                  == false)
                {
                  break;
                }
              dacqh->data_count = measurements;
              if (dacqh->cb != nullptr)
                {
                  dacqh->cb (dacqh);
                }
            }
          result = true;
        }
      while (0);
      mutex_.unlock ();
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
 * @return: Number of characters returned. In case of errors -1 will be returned.
 */
int
sdi12_dr::transaction (char* buff, size_t cmd_len, size_t len)
{
  int result = 0;
  int retries_with_break = 3;
  char answer[SDI12_LONGEST_FRAME];

  do
    {
      // check if we need to send a break
      if (last_sdi_addr_ != buff[0] || (sysclock.now () - last_sdi_time_) > 80)
        {
          // send a break at least 12 ms long
          tty_->tcsendbreak (SDI_BREAK_LEN);
        }
      last_sdi_addr_ = buff[0];     // replace last address

      // wait at least 8.33 ms
      sysclock.sleep_for (10);

      int retries = 3;
      bool valid_sdi12 = false;
      do
        {
          // send request
          if ((result = tty_->write (buff, cmd_len)) < 0)
            {
              break;
            }
          // wait for the end of transmission
          sysclock.sleep_for ((83 * cmd_len) / 10);
          last_sdi_time_ = sysclock.now ();

          // read response, if any
          size_t offset = 0;
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
                }
              // we read until we get a valid frame or overflow the buffer
            }
          while (valid_sdi12 == false && offset < sizeof(answer));
        }
      while (valid_sdi12 == false && --retries);

      if (valid_sdi12)
        {
          strncpy (buff, answer, len);
          last_sdi_time_ = sysclock.now ();
          break;
        }
      else
        {
          // wait a little before a break and new triplet sequence
          sysclock.sleep_for (10);

          // force a break, even if the timeout did not expire
          last_sdi_time_ = 0;
        }
    }
  while (--retries_with_break);

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

  if (sdi->index < 10)
    {
      buff[0] = sdi->addr;
      buff[1] = sdi->method;
      buff[2] = sdi->use_crc ? 'C' : sdi->index ? sdi->index + '0' : '!';
      buff[3] = sdi->use_crc ? (sdi->index ? sdi->index + '0' : '!') : '\0';
      buff[4] = (sdi->use_crc && sdi->index) ? '!' : '\0';
      buff[5] = '\0';

      count = transaction (buff, strlen (buff), sizeof(buff));
      if (count > 6) // we need at least the delay and the number of meas'ments
        {
          buff[count - 2] = '\0';
          measurements = (uint8_t) atoi (&buff[4]);
          buff[4] = '\0';
          response_delay = atoi (&buff[1]);
          result = true;
        }
    }

  return result;
}

/**
 * @brief Wait for a service request from a sensor.
 * @param addr: sensor's address.
 * @param response_delay: number of seconds to wait until the sensor answer,
 *      after which the function returns.
 * @return true if successful, false otherwise.
 * @note The function returns true either if the sensor sent a service request,
 *      or if the timeout expired.
 */
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
              last_sdi_time_ = sysclock.now ();
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

/**
 * @brief Implementation of the "Send Data" command.
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
  char buff[SDI12_LONGEST_FRAME];
  char request = sdi->index + '0';
  uint8_t parsed = 0;
  int count;

  if (data != nullptr && status != nullptr)
    {
      // set all status bytes to "missing values"
      memset (status, STATUS_BIT_MISSING, measurements);
      do
        {
          buff[0] = sdi->addr;
          buff[1] = sdi->method;
          buff[2] = request;
          buff[3] = '!';
          count = transaction (buff, 4, sizeof(buff));

          if (count > 0 && sdi->addr == buff[0])
            {
              if (sdi->use_crc)
                {
                  // verify the CRC
                  char* p = buff + count - 5;  // p points on the first CRC byte
                  uint16_t incoming_crc = (*p++ & 0x3F) << 12;
                  incoming_crc += (*p++ & 0x3F) << 6;
                  incoming_crc += (*p & 0x3F);
                  if (incoming_crc
                      != sdi12_dr::calc_crc (0, (uint8_t*) buff, count - 5))
                    {
                      break;    // invalid CRC
                    }
                }

              char *p, *r = buff + 1; // skip address
              buff[count - (sdi->use_crc ? 5 : 2)] = '\0'; // terminate string
              do
                {
                  p = r;
                  data[parsed] = strtof (p, &r);
                  if (data[parsed] == 0 && p == r)
                    {
                      break;    // conversion to float error, exit
                    }
                  status[parsed] = STATUS_OK;
                  parsed++;
                }
              while (*r != '\0' && parsed < measurements);

              if (sdi->method == sdi12_dr::continuous)
                {
                  break;
                }
            }
        }
      while (request++ < '9' && count > 0 && parsed < measurements);

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
  sdi12_t* sdi = (sdi12_t *) dacqh->impl;

  for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
      if (msgs_[i].sdih.addr == static_cast<sdi12_t*> (dacqh->impl)->addr)
        {
          // this sensor is already in a transaction, abort
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

  // TODO: should we need to kill this task when the dacq object is destroyed?
  return nullptr;
}
#endif // MAX_CONCURRENT_REQUESTS > 0
