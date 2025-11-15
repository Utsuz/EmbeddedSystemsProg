#ifndef USB_H
#define USB_H

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>

// USB init
void usb_init(void);

// Send data via USB
void usb_send(const char *message);

// Send raw data (binary)
void usb_send_raw(const uint8_t *data, size_t length);

// Check if usb is connected
bool usb_is_connected(void);

#endif
