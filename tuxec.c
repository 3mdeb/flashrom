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

#include "chipdrivers.h"
#include "hwaccess.h"
#include "programmer.h"

#define EC_DATA       0x62
#define EC_CONTROL    0x66

#define EC_CMD_WRITE_BLOCK 0x02
#define EC_CMD_READ_BLOCK  0x03
#define EC_CMD_FILL_KBYTE  0x05
#define EC_CMD_FINISH      0xfe

#define EC_STS_IBF      (1 << 1)
#define EC_STS_OBF      (1 << 0)

#define BLOCK_SIZE_IN_BYTES 65536

#define CHUNK_SIZE_IN_BYTES 256
#define CHUNKS_PER_KBYTE 4
#define CHUNKS_PER_BLOCK 256

#define TRY_COUNT  100000

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

static bool tuxec_read_byte(tuxec_data_t *ctx_data, uint8_t *data)
{
	int i;

	for (i = 0; i <= TRY_COUNT; ++i) {
		if ((INB(ctx_data->control_port) & EC_STS_OBF) != 0) {
			break;
		}
	}

	*data = INB(ctx_data->data_port);
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

static int tuxec_probe(struct flashctx *flash)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	flash->chip->feature_bits |= FEATURE_ERASED_ZERO;
	flash->chip->tested = TEST_OK_PREW;
	flash->chip->total_size = ctx_data->flash_size_in_kb;
	flash->chip->block_erasers[0].eraseblocks[0].size = 1024;
	flash->chip->block_erasers[0].eraseblocks[0].count =
		ctx_data->flash_size_in_kb;
	flash->chip->gran = write_gran_1024bytes;
	return 1;
}

static int tuxec_read(struct flashctx *flash, uint8_t *buf,
			  unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int first_byte = start - start%BLOCK_SIZE_IN_BYTES;
	const unsigned int last_byte = ctx_data->flash_size_in_kb*1024;

	unsigned int offset;
	uint8_t rom_byte;

	int ret = 0;

	for (offset = first_byte; offset < last_byte; ++offset) {
		if (offset%BLOCK_SIZE_IN_BYTES == 0) {
			tuxec_write_cmd(ctx_data, EC_CMD_READ_BLOCK);
			tuxec_write_cmd(ctx_data, offset/BLOCK_SIZE_IN_BYTES);
		}

		if (!tuxec_read_byte(ctx_data, &rom_byte)) {
			rom_byte = 0;
			ret = 1;
		}

		if (offset < start) {
			continue;
		}

		*buf = rom_byte;

		++buf;
		--len;

		if (len == 0) {
			break;
		}
	}

	/* Finish reading the block. */
	while (tuxec_read_byte (ctx_data, &rom_byte));

	return ret;
}

static void tuxec_block_write(tuxec_data_t *ctx_data, const uint8_t *buf,
				unsigned int block)
{
	unsigned int i;

	tuxec_write_cmd(ctx_data, EC_CMD_WRITE_BLOCK);
	tuxec_write_cmd(ctx_data, 0x02);
	tuxec_write_cmd(ctx_data, block);
	tuxec_write_cmd(ctx_data, 0x02);
	tuxec_write_cmd(ctx_data, 0x02);

	for (i = 0; i < BLOCK_SIZE_IN_BYTES; ++i) {
		while ((INB(ctx_data->control_port) & EC_STS_IBF) != 0);
		OUTB(*buf, ctx_data->data_port);

		++buf;
	}
}

static int tuxec_write(struct flashctx *flash, const uint8_t *buf,
				unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int first_block = start/BLOCK_SIZE_IN_BYTES;
	const unsigned int last_block = (start + len)/BLOCK_SIZE_IN_BYTES;

	unsigned int block;
	unsigned int offset = start;
	unsigned int left = len;

	int ret = 0;

	for (block = first_block; block < last_block; ++block) {
		const unsigned int block_start = block*BLOCK_SIZE_IN_BYTES;

		uint8_t *tmp_buf;
		unsigned int block_end;
		unsigned int update_size;

		if (offset == block_start && left >= BLOCK_SIZE_IN_BYTES) {
			tuxec_block_write(ctx_data, buf, block);

			offset += BLOCK_SIZE_IN_BYTES;
			left -= BLOCK_SIZE_IN_BYTES;
			continue;
		}

		block_end = (block + 1)*BLOCK_SIZE_IN_BYTES;
		update_size = min(left, block_end - start);

		tmp_buf = (uint8_t *)malloc(BLOCK_SIZE_IN_BYTES);
		if (!tmp_buf) {
			msg_perr("Unable to allocate space for block buffer.\n");
			ret = 1;
			break;
		}

		if (read_opaque(flash,
				tmp_buf,
				block_start,
				BLOCK_SIZE_IN_BYTES)) {
			free(tmp_buf);
			msg_perr("Unable to read block into buffer.\n");
			ret = 1;
		}

		memcpy(tmp_buf + (offset - block_start), buf, update_size);

		tuxec_block_write(ctx_data, tmp_buf, block);

		free(tmp_buf);

		offset += update_size;
		left -= update_size;
	}

	return ret;
}

static int tuxec_erase(struct flashctx *flash,
			unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int first_chunk = start/CHUNK_SIZE_IN_BYTES;
	const unsigned int last_chunk = (start + len)/CHUNK_SIZE_IN_BYTES;

	unsigned int i;

	for (i = first_chunk; i < last_chunk; i += CHUNKS_PER_KBYTE) {
		tuxec_write_cmd(ctx_data, EC_CMD_FILL_KBYTE);
		tuxec_write_cmd(ctx_data, i/CHUNKS_PER_BLOCK);
		tuxec_write_cmd(ctx_data, i%CHUNKS_PER_BLOCK);
		tuxec_write_cmd(ctx_data, 0x00);

		internal_sleep(1000);
	}

	return 0;
}

static struct opaque_master programmer_tuxec = {
	.max_data_read = 65536,
	.max_data_write = 65536,
	.probe		= tuxec_probe,
	.read		= tuxec_read,
	.write		= tuxec_write,
	.erase		= tuxec_erase,
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

	programmer_tuxec.data = ctx_data;

	if (register_shutdown(tuxec_shutdown, ctx_data))
		goto init_err_exit;
	if (register_opaque_master(&programmer_tuxec))
		goto init_err_exit;
	msg_pdbg("%s(): successfully initialized tuxec\n", __func__);

	return 0;

init_err_exit:
	tuxec_shutdown(ctx_data);
	return 1;
}
#endif
