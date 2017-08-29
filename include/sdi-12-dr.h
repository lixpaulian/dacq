/*
 * sdi-12-dr.h
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

#ifndef SDI_12_DR_H_
#define SDI_12_DR_H_

#include <cmsis-plus/rtos/os.h>

#include "uart-drv.h"

#if defined (__cplusplus)

class sdi12_dr
{
public:

  sdi12_dr (const char* name);

  ~sdi12_dr ();

  int
  open (void);

  void
  close (void);

  bool
  ack_active (char addr);

  bool
  send_id (char addr, char* id, size_t id_len);

  bool
  change_address (char addr, char new_addr);

  bool
  start_measurement (char addr, bool concurrent, uint8_t index, bool use_crc,
                     int& response_delay, int& measurements);

  bool
  wait_for_service_request (char addr, int response_delay);

  // --------------------------------------------------------------------

protected:

private:

  static constexpr uint8_t UART_DRV_VERSION_MAJOR = 0;
  static constexpr uint8_t UART_DRV_VERSION_MINOR = 1;

  const char* name_;
  os::posix::tty* tty_;
  char last_sdi_addr_ = '?';
  os::rtos::clock::timestamp_t last_sdi_time_ = 0;

  int
  transaction (char* buff, size_t buff_len);

};

#endif /* (__cplusplus) */

#endif /* SDI_12_DR_H_ */
