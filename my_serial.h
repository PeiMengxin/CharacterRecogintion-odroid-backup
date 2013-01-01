#ifndef MY_SERIAL_H_
#define MY_SERIAL_H_

#include "trace.h"

#define UART_SENT_TYPE_MOUSE 0
#define UART_SENT_TYPE_CHARACTER 1
#define UART_SENT_TYPE_TARGET 2

void uartReadThread();
void enumerate_ports();

void uartSent(int _type);
void Data_pre_character();
void Data_pre_mouse(int x, int y);
void Data_pre_target();

#endif //MY_SERIAL_H_
