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

/*
 * Contains programmer implementation for EC used by Tux laptops.
 */

#if defined(__i386__) || defined(__x86_64__)

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "chipdrivers.h"
#include "hwaccess.h"
#include "programmer.h"

#define EC_DATA             0x62
#define EC_CONTROL          0x66

#define EC_CMD_WRITE_BLOCK  0x02
#define EC_CMD_READ_BLOCK   0x03
#define EC_CMD_FILL_KBYTE   0x05
#define EC_CMD_QUERY        0x80
#define EC_CMD_INIT         0x81
#define EC_CMD_FINISH       0xfe

#define EC_STS_IBF          (1 << 1)
#define EC_STS_OBF          (1 << 0)

#define BYTES_PER_BLOCK     65536
#define BYTES_PER_CHUNK     256
#define KBYTES_PER_BLOCK    64
#define CHUNKS_PER_KBYTE    4
#define CHUNKS_PER_BLOCK    256

#define MAX_STATUS_CHECKS   100000

typedef struct
{
	unsigned int rom_size_in_kbytes;
	uint8_t control_port;
	uint8_t data_port;
} tuxec_data_t;

static bool tuxec_wait_for_ibuf(uint8_t status_port)
{
	int i;

	for (i = 0; (INB(status_port) & EC_STS_IBF) != 0; ++i) {
		if (i == MAX_STATUS_CHECKS) {
			msg_pdbg("%s(): input buf is empty\n", __func__);
			return false;
		}
	}

	return true;
}

static bool tuxec_wait_for_obuf(uint8_t status_port)
{
	int i;

	for (i = 0; (INB(status_port) & EC_STS_OBF) == 0; ++i) {
		if (i == MAX_STATUS_CHECKS) {
			msg_pdbg("%s(): output buf is empty\n", __func__);
			return false;
		}
	}

	return true;
}

static bool tuxec_write_cmd(tuxec_data_t *ctx_data, uint8_t cmd)
{
	const bool success = tuxec_wait_for_ibuf(ctx_data->control_port);
	OUTB(cmd, ctx_data->control_port);
	return success;
}

static bool tuxec_read_byte(tuxec_data_t *ctx_data, uint8_t *data)
{
	const bool success = tuxec_wait_for_obuf(ctx_data->control_port);
	*data = INB(ctx_data->data_port);
	return success;
}

static bool tuxec_write_byte(tuxec_data_t *ctx_data, uint8_t data)
{
	const bool success = tuxec_wait_for_ibuf(ctx_data->control_port);
	OUTB(data, ctx_data->data_port);
	return success;
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
	tuxec_wait_for_ibuf(EC_CONTROL);
	OUTB(EC_CMD_QUERY, EC_CONTROL);

	tuxec_wait_for_ibuf(EC_CONTROL);
	OUTB(data, EC_DATA);

	tuxec_wait_for_ibuf(EC_CONTROL);
	return INB(EC_DATA);
}

static void tuxec_init_ctx(tuxec_data_t *ctx_data)
{
	uint8_t flash_size_in_blocks;

	ctx_data->control_port = EC_CONTROL;
	ctx_data->data_port = EC_DATA;

	switch (tuxec_query(0xf9) & 0xf0) {
	case 0x40:
		flash_size_in_blocks = 3;
		break;
	case 0xf0:
		flash_size_in_blocks = 4;
		break;
	default:
		flash_size_in_blocks = 1;
		break;
	}

	ctx_data->rom_size_in_kbytes = flash_size_in_blocks*KBYTES_PER_BLOCK;
}

static void tuxec_send_init(uint8_t data1, uint8_t data2)
{
	tuxec_wait_for_ibuf(EC_CONTROL);
	OUTB(EC_CMD_INIT, EC_CONTROL);

	tuxec_wait_for_ibuf(EC_CONTROL);
	OUTB(data1, EC_DATA);

	tuxec_wait_for_ibuf(EC_CONTROL);
	OUTB(data2, EC_DATA);
}

static int tuxec_probe(struct flashctx *flash)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	flash->chip->feature_bits |= FEATURE_ERASED_ZERO;
	flash->chip->tested = TEST_OK_PREW;
	flash->chip->total_size = ctx_data->rom_size_in_kbytes;
	flash->chip->gran = write_gran_1024bytes;

	flash->chip->block_erasers[0].eraseblocks[0].size = 1024;
	flash->chip->block_erasers[0].eraseblocks[0].count =
		ctx_data->rom_size_in_kbytes;

	return 1;
}

static int tuxec_read(struct flashctx *flash, uint8_t *buf,
			  unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int first_byte = start - start%BYTES_PER_BLOCK;
	const unsigned int last_byte = ctx_data->rom_size_in_kbytes*1024;

	unsigned int offset;
	uint8_t rom_byte;

	int ret = 0;

	for (offset = first_byte; offset < last_byte; ++offset) {
		if (offset%BYTES_PER_BLOCK == 0) {
			tuxec_write_cmd(ctx_data, EC_CMD_READ_BLOCK);
			tuxec_write_cmd(ctx_data, offset/BYTES_PER_BLOCK);
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
	while (tuxec_read_byte(ctx_data, &rom_byte)) {
		continue;
	}

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

	for (i = 0; i < BYTES_PER_BLOCK; ++i) {
		tuxec_write_byte(ctx_data, *buf);
		++buf;
	}
}

static int tuxec_write(struct flashctx *flash, const uint8_t *buf,
				unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int first_block = start/BYTES_PER_BLOCK;
	const unsigned int last_block = (start + len)/BYTES_PER_BLOCK;

	unsigned int block;
	unsigned int offset = start;
	unsigned int left = len;

	int ret = 0;

	for (block = first_block; block < last_block; ++block) {
		const unsigned int block_start = block*BYTES_PER_BLOCK;

		uint8_t *tmp_buf;
		unsigned int block_end;
		unsigned int update_size;

		if (offset == block_start && left >= BYTES_PER_BLOCK) {
			tuxec_block_write(ctx_data, buf, block);

			offset += BYTES_PER_BLOCK;
			left -= BYTES_PER_BLOCK;
			continue;
		}

		block_end = (block + 1)*BYTES_PER_BLOCK;
		update_size = min(left, block_end - start);

		tmp_buf = (uint8_t *)malloc(BYTES_PER_BLOCK);
		if (!tmp_buf) {
			msg_perr("Unable to allocate space for block "
					"buffer.\n");
			ret = 1;
			break;
		}

		if (read_opaque(flash, tmp_buf, block_start, BYTES_PER_BLOCK)) {
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

	const unsigned int first_chunk = start/BYTES_PER_CHUNK;
	const unsigned int last_chunk = (start + len)/BYTES_PER_CHUNK;

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
		msg_pdbg("%s(): tuxec only supports \"ec\" type devices\n",
				__func__);
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
