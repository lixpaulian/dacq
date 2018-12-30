/*
 * dacq.h
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

#ifndef DACQ_H_
#define DACQ_H_

#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/posix/termios.h>

#include "uart-drv.h"
#include "dacq-config.h"

#if defined (__cplusplus)

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
    uint8_t data_count; // number of expected/returned values (tags)
    void* impl;         // pointer to a struct, implementation specific
    bool
    (*cb) (void *);     // user call-back function to handle data
    void* cb_parameter; // pointer on a custom parameter for the call-back
  } dacq_handle_t;

  /**
   * @brief Open a DACQ serial port; all parameters as per the definitions
   *    in termios.h
   * @param baudrate: baud rate.
   * @param c_size: character size (CS5, CS6, CS7 or CS8).
   * @param parity: parity (0 or PARENB | PARODD)
   * @param rec_timeout: receive timeout, in ms.
   * @return true if successful, false otherwise.
   */
  bool
  open (speed_t baudrate, uint32_t c_size, uint32_t parity,
        uint32_t rec_timeout);

  /**
   * @brief Provides a direct connection to the DACQ port.
   * @param fildes: a file descriptor of a stream that will directly
   *    communicate with the DACQ port.
   */
  void
  direct (int fildes);

  /**
   * @brief Close the DAQC serial port.
   */
  void
  close (void);

  /**
   * @brief Check if an operation is under way i.e. if the system is busy.
   * @return true if busy, false otherwise.
   */
  bool
  is_busy (void);

  /**
   * @brief Return the driver's version.
   * @param version_major: major version number.
   * @param version_minor: minor version number.
   */
  virtual void
  get_version (uint8_t& version_major, uint8_t& version_minor) = 0;

  /**
   * @brief Get info about the sensor/logger (version, manufacturer, etc.).
   * @param id: sensor ID (or address).
   * @param info: buffer where the information will be returned.
   * @param len: size of the info buffer.
   * @return true if successful, false otherwise.
   */
  virtual bool
  get_info (int id, char* info, size_t len) = 0;

  /**
   * @brief Retrieve data from sensor or data logger.
   * @param dacqh: pointer to a dacq_handle_t structure (see above).
   * @return true if successful, false otherwise.
   */
  virtual bool
  retrieve (dacq_handle_t* dacqh) = 0;

  /**
   * @brief Execute a transparent operation (request/answer).
   * @param xfer_buff: buffer containing the request and returning the answer.
   * @param len: on entry, the buffer length; on exit the answer's length.
   * @return true if successful, false otherwise.
   */
  virtual bool
  transparent (char* xfer_buff, int& len);

  /**
   * @brief Change sensor/logger ID (address, etc.).
   * @param id: current ID.
   * @param new_id: new ID.
   * @return true if successful, false otherwise.
   */
  virtual bool
  change_id (int id, int new_id);

  /**
   * @brief Set the data acquisition interval (also "sampling interval").
   * @param interval: reference to the data acquisition interval, in seconds.
   * @return true if successful, false otherwise.
   */
  virtual bool
  set_acq_interval (int interval);

  /**
   * @brief Get the sensor's data acquisition interval.
   * @param interval: reference to the returned data acquisition interval.
   * @return true if successful, false otherwise.
   */
  virtual bool
  get_acq_interval (int& interval);

  /**
   * @brief Set the internal clock of the sensor/data logger.
   * @param date: date to be set.
   * @return true if successful, false otherwise.
   */
  virtual bool
  set_date (time_t date);

  /**
   * @brief Get the current clock value of the sensor/logger.
   * @return the current date.
   */
  virtual time_t
  get_date (void);

  /**
   * @brief Abort a retrieve operation.
   * @return true if successful, false otherwise.
   */
  virtual bool
  abort (void);

  // sensor status values
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_BIT_MISSING = 1;
  static constexpr uint8_t STATUS_BIT_IMPLAUSIBILE = 2;

  typedef enum
  {
    ok = 0, tty_in_use, tty_open, tty_attr, dacq_busy,

    //
    timeout,
    unexpected_answer,
    sensor_busy,
    too_many_requests,
    invalid_index,
    crc_error,
    conversion_to_float_error,
    no_sensor_data,
    set_time_error,
    buffer_too_small,
    set_acq_interval_failed,
    initialisation_required,

    //
    last

  } err_num_t;

  typedef struct err_
  {
    err_num_t error_number;
    const char* error_text;
  } err_t;

  const err_t* error = &err_[ok];

protected:

  os::posix::tty* tty_;
  os::posix::tty* console_;
  os::rtos::mutex mutex_
    { "dacq_mx" };

  // define a millisecond based on the scheduler's frequency
  static constexpr uint32_t one_ms = 1000 / os::rtos::sysclock.frequency_hz;

  // dacq common errors; the order is important, must be the same as the
  // order in the err_common_t enum.
  err_t err_[last] =
    {
      { ok, "OK" },
      { tty_in_use, "tty already in use" },
      { tty_open, "could not open tty" },
      { tty_attr, "could not set tty attributes" },
      { dacq_busy, "timeout, dacq system busy" },

      { timeout, "sensor timed out" },
      { unexpected_answer, "unexpected answer" },
      { sensor_busy, "sensor busy" },
      { too_many_requests, "too many concurrent requests" },
      { invalid_index, "invalid index" },
      { crc_error, "crc error" },
      { conversion_to_float_error, "conversion to float error" },
      { no_sensor_data, "no valid data from sensor" },
      { set_time_error, "failed to set date/time on sensor/logger" },
      { buffer_too_small, "return buffer too small" },
      { set_acq_interval_failed, "failed to set the acquisition interval" },
      { initialisation_required, "sensor/logger requires initialisation" },

    };

private:

  static void*
  dacq_rcv (void* args);

  const char* name_;

};

inline bool
dacq::is_busy (void)
{
  return (mutex_.owner () != nullptr);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// Following functions might, or might not be supported by all sensors and/or
// data loggers; derived classes should override them as required.

inline bool
dacq::transparent (char* xfer_buff, int& len)
{
  return false;
}

inline bool
dacq::change_id (int id, int new_id)
{
  return false;
}

inline bool
dacq::set_acq_interval (int interval)
{
  return false;
}

inline bool
dacq::get_acq_interval (int& interval)
{
  interval = 0;
  return false;
}

inline bool
dacq::set_date (time_t date)
{
  return false;
}

inline time_t
dacq::get_date (void)
{
  return 0;
}

inline bool
dacq::abort (void)
{
  return false;
}

#pragma GCC diagnostic pop

#endif /* (__cplusplus) */

#endif /* DACQ_H_ */
