/*
 * sdi-12-dr-test.cpp
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

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmsis-plus/rtos/os.h>
#include <cmsis-plus/diag/trace.h>
#include <cmsis-plus/posix-io/file-descriptors-manager.h>

#include "uart-drv.h"
#include "sdi-12-dr.h"
#include "io.h"
#include "sysconfig.h"
#include "sdi-12-dr-test.h"

#if SDI12_TEST == true

extern "C"
{
  UART_HandleTypeDef huart1;
}

using namespace os;

// Static manager
os::posix::file_descriptors_manager descriptors_manager
  { 8 };

#define TX_BUFFER_SIZE 200
#define RX_BUFFER_SIZE 200
#define SDI_BUFF_SIZE 84

sdi12_uart uart1
  { "uart1", &huart1, nullptr, nullptr, TX_BUFFER_SIZE, RX_BUFFER_SIZE, true,
      driver::uart::RS485_DE_POLARITY_MASK };

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

#if MAX_CONCURENT_REQUESTS != 0
static bool
cb_get_data (char addr, float* data, int max_values);
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

  do
    {
      // open sdi12 port
      if (sdi12dr.open () == false)
        {
          trace::printf ("Could not open sdi12 port\n");
          break;
        }
      trace::printf ("sdi12 port opened\n", sensor_addr);

      // send acknowledge active command (a!)
      if (sdi12dr.ack_active (sensor_addr) == false)
        {
          trace::printf ("Sensor %c not responding\n", sensor_addr);
          break;
        }
      trace::printf ("Sensor %c is active\n", sensor_addr);

      // identification command (aI!)
      if (sdi12dr.send_id (sensor_addr, buff, sizeof(buff)) == false)
        {
          trace::printf ("Failed to get sensor identification\n");
          break;
        }
      trace::printf ("Sensor identification: %s\n", buff);

      // change address from 0 to 1 (aAb!)
      if (sdi12dr.change_address (sensor_addr, '1') == false)
        {
          trace::printf ("Failed to change sensor's address\n");
          break;
        }
      sensor_addr = '1';
      trace::printf ("Sensor address changed to %c\n", sensor_addr);

      // measure (M/C/R/V with D)
      int meas;
      float data[20];
      meas = 20;
      if (sdi12dr.sample_sensor (sensor_addr, sdi12_dr::measure, 0, false,
                                 data, meas) == false)
        {
          trace::printf ("Error getting data from sensor\n");
          break;
        }
      trace::printf ("Got %d values from sensor\n", meas);
      for (int i = 0; i < meas; i++)
        {
          trace::printf ("%f, ", data[i]);
        }
      trace::printf ("\n");

#if MAX_CONCURENT_REQUESTS != 0
      // asynchronous measure (C with D)
      if (sdi12dr.sample_sensor_async (sensor_addr, 0, false,
                                 data, meas, cb_get_data) == false)
        {
          trace::printf ("Error getting concurrent data from sensor\n");
          break;
        }

      // wait for the asynchronous measurement to finish
      rtos::sysclock.sleep_for (4000);
#endif // MAX_CONCURENT_REQUESTS != 0

      // change address to original address
      if (sdi12dr.change_address (sensor_addr, '0') == false)
        {
          trace::printf ("Failed to change sensor's address\n");
          break;
        }
      sensor_addr = '0';
      trace::printf ("Sensor address changed back to %c\n", sensor_addr);

      result = true;
    }
  while (0);

  // close sdi12 port
  sdi12dr.close ();
  trace::printf ("sdi12 port closed\n", sensor_addr);

  if (result == false)
    {
      trace::printf ("SDI-12 test failed\n");
    }
  else
    {
      trace::printf ("SDI-12 test successful\n");
    }
}

#if MAX_CONCURENT_REQUESTS != 0
static bool
cb_get_data (char addr, float* data, int max_values)
{
  trace::printf ("Got %d values from sensor %c\n", max_values, addr);
  for (int i = 0; i < max_values; i++)
    {
      trace::printf ("%f, ", data[i]);
    }
  trace::printf ("\n");
  return true;
}
#endif // MAX_CONCURENT_REQUESTS != 0
#endif
