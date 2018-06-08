/*
 * sdi-12-uart.cpp
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
 * Created on: 4 Sep 2017 (LNP)
 */

#include <cmsis-plus/rtos/os.h>
#include "sdi-12-uart.h"

using namespace os;
using namespace os::rtos;
using namespace os::driver::stm32f7;

// implementation of our own sdi12 uart derived class

sdi12_uart_impl::sdi12_uart_impl (UART_HandleTypeDef* huart,
                        uint8_t* tx_buff, uint8_t* rx_buff, size_t tx_buff_size,
                        size_t rx_buff_size, uint32_t rs485_params) :
    uart_impl
      { huart, tx_buff, rx_buff, tx_buff_size, rx_buff_size, rs485_params }
{
  trace::printf ("%s() %p\n", __func__, this);
}

sdi12_uart_impl::~sdi12_uart_impl ()
{
  trace::printf ("%s() %p\n", __func__, this);
}

int
sdi12_uart_impl::do_tcsendbreak (int duration)
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
sdi12_uart_impl::do_rs485_de (bool state)
{
  if (rs485_params_)
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

