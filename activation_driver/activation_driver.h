#ifndef ACTIVATION_DRIVER_H
#define ACTIVATION_DRIVER_H

#include <stdint.h>
#include <stddef.h>

void activation_driver_init(int uart_num, int tx_pin, int rx_pin, uint32_t baudrate);
void activation_send(const char *message);
int  activation_receive(char *buffer, size_t max_len);
int activation_receive_line(char *buffer, size_t max_len);

#endif

