/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2021, 3mdeb Embedded Systems Consulting
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __EC_H__
#define __EC_H__ 1

#include <stdbool.h>
#include <stdint.h>

/*
 * Generic IO functions for ACPI-compliant embedded controllers
 */

/* Standard ports */
#define EC_DATA     0x62
#define EC_CONTROL  0x66 /* Read status, write commands */

/* How many iterations to wait for input or output buffer */
#define EC_MAX_STATUS_CHECKS  100000

bool ec_wait_for_ibuf(uint8_t control_port);
bool ec_wait_for_obuf(uint8_t control_port, unsigned int max_checks);

bool ec_write_cmd(uint8_t control_port, uint8_t cmd);
bool ec_read_byte(uint8_t control_port, uint8_t data_port, uint8_t *data);
bool ec_write_byte(uint8_t control_port, uint8_t data_port, uint8_t data);

/* These implement standard ACPI commands and thus use standard ports */
bool ec_read_reg(uint8_t address, uint8_t *data);
bool ec_write_reg(uint8_t address, uint8_t data);

#endif		/* !__EC_H__ */
