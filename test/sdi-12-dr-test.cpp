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
#define SDI_BUFF_SIZE 80

sdi12_uart uart1
  { "uart1", &huart1, nullptr, nullptr, TX_BUFFER_SIZE, RX_BUFFER_SIZE, true,
      0x80000000 };

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

/**
 * @brief  This is a test function that exercises the SDI-12 library.
 */
void
test_sdi12 (void)
{
  char buff[100];
  int result = -1;
  char sensor_addr = '0';

  do
    {
      if (sdi12dr.open () < 0)
        {
          trace::printf ("Could not open sdi12 port\n", sensor_addr);
          break;
        }
      trace::printf ("sdi12 port opened\n", sensor_addr);

      if (sdi12dr.ack_active (sensor_addr) == false)
        {
          trace::printf ("Sensor %c not responding\n", sensor_addr);
          break;
        }
      trace::printf ("Sensor %c is active\n", sensor_addr);

      if (sdi12dr.send_id (sensor_addr, buff, sizeof(buff)) == false)
        {
          trace::printf ("Failed to get sensor identification\n");
          break;
        }
      trace::printf ("Sensor identification: %s\n", buff);

      if (sdi12dr.change_address (sensor_addr, '1') == false)
        {
          trace::printf ("Failed to change sensor's address\n");
          break;
        }
      sensor_addr = '1';
      trace::printf ("Sensor address changed to %c\n", sensor_addr);

      int delay;
      int meas;
      if (sdi12dr.start_measurement (sensor_addr, delay, meas) == false)
        {
          trace::printf ("Failed to start a measurement\n");
          break;
        }
      trace::printf ("Expect %d values after %d seconds\n", meas, delay);

      if (sdi12dr.wait_for_service_request (delay) == false)
        {
          trace::printf ("Error waiting for the sensor\n");
          break;
        }

      if (sdi12dr.change_address (sensor_addr, '0') == false)
        {
          trace::printf ("Failed to change sensor's address\n");
          break;
        }
      sensor_addr = '0';
      trace::printf ("Sensor address changed back to %c\n", sensor_addr);

      sdi12dr.close ();
      trace::printf ("sdi12 port closed\n", sensor_addr);
      result = 0;
    }
  while (0);

  if (result < 0)
    {
      trace::printf ("SDI-12 test failed\n");
    }
  else
    {
      trace::printf ("SDI-12 test successful\n");
    }

}

// implementation of our own sdi12 uart derived class

sdi12_uart::sdi12_uart (const char* name, UART_HandleTypeDef* huart,
                        uint8_t* tx_buff, uint8_t* rx_buff, size_t tx_buff_size,
                        size_t rx_buff_size, bool is_rs485,
                        uint32_t rs485_de_params) :
    uart
      { name, huart, tx_buff, rx_buff, tx_buff_size, rx_buff_size, is_rs485,
          rs485_de_params }
{
  trace::printf ("%s() %p\n", __func__, this);
}

sdi12_uart::~sdi12_uart ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

int
sdi12_uart::do_tcsendbreak (int duration)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  do_rs485_de (true);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  rtos::sysclock.sleep_for (duration);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  do_rs485_de (false);

  return 0;
}

void
sdi12_uart::do_rs485_de (bool state)
{
  if (rs485_de_params_)
    {
      HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4 | GPIO_PIN_5,
                         state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  else
    {
      HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4 | GPIO_PIN_5,
                         state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

#endif
