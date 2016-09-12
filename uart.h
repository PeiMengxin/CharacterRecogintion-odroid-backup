/*
 * uart.h
 *
 *  Created on: May 5, 2016
 *      Author: odroid
 */

#ifndef UART_H_
#define UART_H_


void uart_test();
void uart_send();
void read_uart();
void uart_mouse(int x, int y);
void uart_depth(int x[], int x_size);
extern int flow_pix[2],uart_good,fd;
#endif /* UART_H_ */
