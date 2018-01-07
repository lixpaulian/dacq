/*
 * sdi-12-dr-test.cpp
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


#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/posix-io/file-descriptors-manager.h>
#include <cmsis-plus/diag/trace.h>

#include "test-sdi12dr.h"
#include "sdi-12-dr.h"
#include "sysconfig.h"

#if SDI12_TEST == true

extern "C"
{
  UART_HandleTypeDef huart1;
}

using namespace os;

// Static manager
os::posix::file_descriptors_manager descriptors_manager
  { 8 };

#define TX_BUFFER_SIZE 100
#define RX_BUFFER_SIZE 100
#define SDI_BUFF_SIZE 84

sdi12_uart uart1
  { "uart1", &huart1, nullptr, nullptr, TX_BUFFER_SIZE, RX_BUFFER_SIZE,
      driver::uart::RS485_MASK | driver::uart::RS485_DE_POLARITY_MASK };

void
HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
    {
      uart1.cb_tx_event ();
    }
}

void
HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
    {
      uart1.cb_rx_event (false);
    }
}

void
HAL_UART_RxHalfCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
    {
      uart1.cb_rx_event (true);
    }
}

void
HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
    {
      uart1.cb_rx_event (false);
    }
}

sdi12_dr sdi12dr
  { "/dev/uart1" };

#if MAX_CONCURRENT_REQUESTS > 0
static bool
cb_get_data (dacq::dacq_handle_t* dacqh);
#endif

/**
 * @brief  This is a test function that exercises the SDI-12 library.
 */
void
test_sdi12 (void)
{
  char buff[100];
  bool result = false;
  char sensor_addr = '0';

  class dacq* dacqp = &sdi12dr;

  do
    {
      uint8_t version_major, version_minor;

      dacqp->get_version (version_major, version_minor);
      trace::printf ("SDI-12 driver version: %d.%d\n", version_major,
                     version_minor);

      // open sdi12 port: 1200 Baud, 7 bits, even parity, 50 ms timeout
      if (dacqp->open (1200, CS7, PARENB, 50) == false)
        {
          trace::printf ("Serial port: %s\n", dacqp->error->error_text);
          break;
        }
      trace::printf ("sdi12 port opened\n", sensor_addr);

      // identification command (aI!)
      if (dacqp->get_info (sensor_addr, buff, sizeof(buff)) == false)
        {
          trace::printf ("Get sensor ID: %s\n", dacqp->error->error_text);
          break;
        }
      trace::printf ("Sensor ID: %s\n", buff);

      // change address from 0 to 1
      int new_addr = '1';
      if (dacqp->change_id (sensor_addr, new_addr) == false)
        {
          trace::printf ("Address change failed: %s\n",
                         dacqp->error->error_text);
          break;
        }
      sensor_addr = '1';
      trace::printf ("Sensor address changed to %c\n", sensor_addr);

      // measure (M with D)
      float data[20];
      uint8_t status[20];
      dacq::dacq_handle_t dacqh;
      dacqh.data = data;
      dacqh.status = status;
      dacqh.data_count = sizeof(data);
      dacqh.cb = nullptr;
      dacqh.user_process = nullptr;
      sdi12_dr::sdi12_t sdi;
      dacqh.impl = (void *) &sdi;
      sdi.addr = sensor_addr;
      sdi.method = sdi12_dr::measure;
      sdi.index = 0;
      sdi.use_crc = false;
      if (dacqp->retrieve (&dacqh) == false)
        {
          trace::printf ("Error getting data from sensor: %s\n",
                         dacqp->error->error_text);
          break;
        }
      trace::printf ("Got %d values from sensor\n", dacqh.data_count);
      for (int i = 0; i < dacqh.data_count; i++)
        {
          trace::printf ("%f[%d] ", data[i], status[i]);
        }
      trace::printf ("\n");

#if MAX_CONCURRENT_REQUESTS > 0
      // asynchronous measure (C with D)
      sdi.method = sdi12_dr::concurrent;
      dacqh.data_count = sizeof(data);
      dacqh.cb = cb_get_data;
      if (dacqp->retrieve (&dacqh) == false)
        {
          trace::printf ("Error getting concurrent data from sensor %c: %s\n",
                         sdi.addr, dacqp->error->error_text);
          break;
        }

      // sample now sensor with address A
      sdi.addr = 'A';
      sdi.index = 0;
      sdi.use_crc = true;
      dacqh.data_count = sizeof(data);
      if (dacqp->retrieve (&dacqh) == false)
        {
          trace::printf ("Error getting concurrent data from sensor %c: %s\n",
                         sdi.addr, dacqp->error->error_text);
          break;
        }

      // wait for the asynchronous measurement to finish
      rtos::sysclock.sleep_for (5000 * 1000 / sysclock.frequency_hz);
#endif // MAX_CONCURRENT_REQUESTS > 0

      // change address to original address
      new_addr = '0';
      if (dacqp->change_id (sensor_addr, new_addr) == false)
        {
          trace::printf ("Address change failed: %s\n",
                         dacqp->error->error_text);
          break;
        }
      sensor_addr = '0';
      trace::printf ("Sensor address changed back to %c\n", sensor_addr);

      // close sdi12 port
      sdi12dr.close ();
      trace::printf ("sdi12 port closed\n", sensor_addr);

      result = true;
    }
  while (0);

  if (result == false)
    {
      trace::printf ("SDI-12 test failed\n");
    }
  else
    {
      trace::printf ("SDI-12 test successful\n");
    }
}

#if MAX_CONCURRENT_REQUESTS > 0
static bool
cb_get_data (dacq::dacq_handle_t* dacqh)
{
  trace::printf ("Got %d values from sensor %c\n", dacqh->data_count,
                 static_cast<sdi12_dr::sdi12_t*> (dacqh->impl)->addr);
  for (int i = 0; i < dacqh->data_count; i++)
    {
      trace::printf ("%f[%d] ", dacqh->data[i], dacqh->status[i]);
    }
  trace::printf ("\n");
  return true;
}
#endif // MAX_CONCURRENT_REQUESTS != 0
#endif
