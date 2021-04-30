/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2010-2020, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *    * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#if defined(__i386__) || defined(__x86_64__)
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "hwaccess.h"
#include "programmer.h"
#include "spi.h"

#define EC_DATA       0x62
#define EC_CONTROL    0x66

#define EC_CMD_FINISH 0xfe

#define EC_STS_IGN1    (1 << 7)
#define EC_STS_SMI_EVT (1 << 6)
#define EC_STS_SCI_EVT (1 << 5)
#define EC_STS_BURST   (1 << 4)
#define EC_STS_CMD     (1 << 3)
#define EC_STS_IGN2    (1 << 2)
#define EC_STS_IBF     (1 << 1)
#define EC_STS_OBF     (1 << 0)

#define TRY_COUNT 100000

typedef struct
{
	uint8_t flash_size_in_kb;
	uint8_t flash_size_in_blocks;
	uint8_t control_port;
	uint8_t data_port;
} tuxec_data_t;

static bool tuxec_write_cmd(tuxec_data_t *ctx_data, uint8_t cmd)
{
	int i;

	for (i = 0; i <= TRY_COUNT; ++i) {
		if ((INB(ctx_data->control_port) & EC_STS_IBF) == 0) {
			break;
		}
	}

	OUTB(cmd, ctx_data->control_port);
	return (i <= TRY_COUNT);
}

static int tuxec_shutdown(void *data)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)data;

	tuxec_write_cmd(ctx_data, EC_CMD_FINISH);

	free(data);
	return 0;
}

static uint8_t tuxec_query(uint8_t data)
{
	int i;

	for (i = 0; i < TRY_COUNT; ++i) {
		if ((INB(EC_CONTROL) & EC_STS_IBF) == 0) {
			break;
		}
	}
	OUTB(0x80, EC_CONTROL);

	while ((INB(EC_CONTROL) & EC_STS_IBF) != 0);
	OUTB(data, EC_DATA);

	for (i = 0; i < TRY_COUNT; ++i) {
		if ((INB(EC_CONTROL) & EC_STS_IBF) == 0) {
			break;
		}
	}
	return INB(EC_DATA);
}

static void tuxec_init_ctx(tuxec_data_t *ctx_data)
{
	ctx_data->control_port = EC_CONTROL;
	ctx_data->data_port = EC_DATA;

	switch (tuxec_query(0xf9) & 0xf0) {
	case 0x40:
		ctx_data->flash_size_in_kb = 192;
		ctx_data->flash_size_in_blocks = 3;
		break;
	case 0xf0:
		ctx_data->flash_size_in_kb = 255;
		ctx_data->flash_size_in_blocks = 4;
		break;
	default:
		ctx_data->flash_size_in_kb = 128;
		ctx_data->flash_size_in_blocks = 1;
		break;
	}
}

static void tuxec_send_init(uint8_t data1, uint8_t data2)
{
	int i;

	for (i = 0; i <= TRY_COUNT; ++i) {
		if ((INB(EC_CONTROL) & EC_STS_IBF) == 0) {
			break;
		}
	}
	OUTB(0x81, EC_CONTROL);

	while ((INB(EC_CONTROL) & EC_STS_IBF) != 0);
	OUTB(data1, EC_DATA);

	while ((INB(EC_CONTROL) & EC_STS_IBF) != 0);
	OUTB(data2, EC_DATA);
}

static int tuxec_spi_send_command(const struct flashctx *flash,
								  unsigned int writecnt,
                                  unsigned int readcnt,
                                  const unsigned char *writearr,
                                  unsigned char *readarr)
{
	int ret = 0;
	return ret;
}

static struct spi_master spi_master_tuxec = {
	.max_data_read = 65536,
	.max_data_write = 65536,
	.command = tuxec_spi_send_command,
	.multicommand = default_spi_send_multicommand,
	.read = default_spi_read,
	.write_256 = default_spi_write_256,
};

static int tuxec_check_params(void)
{
	int ret = 0;
	char *const p = extract_programmer_param("type");
	if (p && strcmp(p, "ec")) {
		msg_pdbg("tuxec only supports \"ec\" type devices\n");
		ret = 1;
	}

	free(p);
	return ret;
}

int tuxec_init(void)
{
	tuxec_data_t *ctx_data = NULL;

	msg_pdbg("%s(): entered\n", __func__);

	if (tuxec_check_params())
		return 1;

	ctx_data = calloc(1, sizeof(tuxec_data_t));
	if (!ctx_data) {
		msg_perr("Unable to allocate space for extra context data.\n");
		return 1;
	}

	tuxec_send_init(0xf9, 0x20);
	tuxec_send_init(0xfa, 0x02);
	tuxec_send_init(0xfb, 0x00);
	tuxec_send_init(0xf8, 0xb1);

	tuxec_init_ctx(ctx_data);

	spi_master_tuxec.data = ctx_data;

	if (register_shutdown(tuxec_shutdown, ctx_data))
		goto init_err_exit;
	if (register_spi_master(&spi_master_tuxec))
		goto init_err_exit;
	msg_pdbg("%s(): successfully initialized tuxec\n", __func__);

	return 0;

init_err_exit:
	tuxec_shutdown(ctx_data);
	return 1;
}
#endif
