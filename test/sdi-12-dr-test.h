/*
 * sdi-12-dr-test.h
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

#ifndef TEST_SDI_12_DR_TEST_H_
#define TEST_SDI_12_DR_TEST_H_

#include <cmsis-plus/rtos/os.h>

#include "uart-drv.h"

#if defined (__cplusplus)

void
test_sdi12 (void);

// we derive own uart class to customize the rs485 DE signal and the
// send break timing
class sdi12_uart : public os::driver::uart
{
public:

  sdi12_uart (const char* name, UART_HandleTypeDef* huart, uint8_t* tx_buff,
              uint8_t* rx_buff, size_t tx_buff_size, size_t rx_buff_size,
              bool is_rs485, uint32_t rs485_de_params);

  virtual
  ~sdi12_uart () noexcept;

protected:

  virtual int
  do_tcsendbreak (int duration) override;

  virtual void
  do_rs485_de (bool state) override;

};

#endif /* (__cplusplus) */

#endif /* TEST_SDI_12_DR_TEST_H_ */
