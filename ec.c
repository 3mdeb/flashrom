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

#include "ec.h"

#include <stdbool.h>
#include <stdint.h>

#include "hwaccess.h"
#include "flash.h"

/* Standard commands */
#define EC_CMD_READ_REG   0x80 /* Read register's value */
#define EC_CMD_WRITE_REG  0x81 /* Write register's value */

/* Some of the status bits */
#define EC_STS_IBF  (1 << 1) /* EC's input buffer full (host can't write) */
#define EC_STS_OBF  (1 << 0) /* EC's output buffer full (host can read) */

bool ec_wait_for_ibuf(uint8_t control_port)
{
	unsigned int i;

	for (i = 0; (INB(control_port) & EC_STS_IBF) != 0; ++i) {
		if (i == EC_MAX_STATUS_CHECKS) {
			msg_pdbg("%s(): input buf is not empty\n", __func__);
			return false;
		}
	}

	return true;
}

bool ec_wait_for_obuf(uint8_t control_port, unsigned int max_checks)
{
	unsigned int i;

	for (i = 0; (INB(control_port) & EC_STS_OBF) == 0; ++i) {
		if (i == max_checks) {
			msg_pdbg("%s(): output buf is empty\n", __func__);
			return false;
		}
	}

	return true;
}

bool ec_write_cmd(uint8_t control_port, uint8_t cmd)
{
	const bool success = ec_wait_for_ibuf(control_port);
	if (success) {
		OUTB(cmd, control_port);
	}
	return success;
}

bool ec_read_byte(uint8_t control_port, uint8_t data_port, uint8_t *data)
{
	const bool success = ec_wait_for_obuf(control_port, EC_MAX_STATUS_CHECKS);
	if (success) {
		*data = INB(data_port);
	}
	return success;
}

bool ec_write_byte(uint8_t control_port, uint8_t data_port, uint8_t data)
{
	const bool success = ec_wait_for_ibuf(control_port);
	if (success) {
		OUTB(data, data_port);
	}
	return success;
}

bool ec_read_reg(uint8_t address, uint8_t *data)
{
	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(EC_CMD_READ_REG, EC_CONTROL);

	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(address, EC_DATA);

	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	*data = INB(EC_DATA);

	return true;
}

bool ec_write_reg(uint8_t address, uint8_t data)
{
	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(EC_CMD_WRITE_REG, EC_CONTROL);

	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(address, EC_DATA);

	if (!ec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(data, EC_DATA);

	return true;
}
