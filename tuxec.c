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
 * Contains programmer implementation for EC used by Tuxedo laptops.
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
#define EC_CMD_ERASE_KBYTE  0x05
#define EC_CMD_READ_REG     0x80
#define EC_CMD_WRITE_REG    0x81
#define EC_CMD_FINISH       0xfe

#define EC_STS_IBF          (1 << 1)
#define EC_STS_OBF          (1 << 0)

#define BYTES_PER_BLOCK     64*1024
#define BYTES_PER_CHUNK     256
#define KBYTES_PER_BLOCK    64
#define CHUNKS_PER_KBYTE    4
#define CHUNKS_PER_BLOCK    256

#define MAX_STATUS_CHECKS   100000

enum autoloadaction {
	AUTOLOAD_NO_ACTION,
	AUTOLOAD_DISABLE,
	AUTOLOAD_SETON,
	AUTOLOAD_SETOFF
};


enum autoloadpatchstate {
	AUTOLOAD_NONE,
	AUTOLOAD_ONE_BYTE,
	AUTOLOAD_TWO_BYTES,
	AUTOLOAD_THREE_BYTES
};

typedef struct
{
	unsigned int rom_size_in_blocks;
	unsigned int rom_size_in_kbytes;

	unsigned int autoload_offset; /* meaningful if state != AUTOLOAD_NONE */
	enum autoloadpatchstate autoload_state;
	enum autoloadaction autload_action;

	uint8_t control_port;
	uint8_t data_port;
	bool ac_adapter_plugged;
} tuxec_data_t;

static bool tuxec_wait_for_ibuf(uint8_t status_port)
{
	int i;

	for (i = 0; (INB(status_port) & EC_STS_IBF) != 0; ++i) {
		if (i == MAX_STATUS_CHECKS) {
			msg_pdbg("%s(): input buf is not empty\n", __func__);
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

	if (!tuxec_write_cmd(ctx_data, EC_CMD_FINISH))
		msg_pdbg("%s(): failed to deinitialize controller\n", __func__);

	free(data);
	return 0;
}

static bool tuxec_read_reg(uint8_t address, uint8_t *data)
{
	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(EC_CMD_READ_REG, EC_CONTROL);

	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(address, EC_DATA);

	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	*data = INB(EC_DATA);

	return true;
}

static bool tuxec_write_reg(uint8_t address, uint8_t data)
{
	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(EC_CMD_WRITE_REG, EC_CONTROL);

	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(address, EC_DATA);

	if (!tuxec_wait_for_ibuf(EC_CONTROL))
		return false;
	OUTB(data, EC_DATA);

	return true;
}

static bool tuxec_init_ctx(tuxec_data_t *ctx_data)
{
	uint8_t reg_value;

	ctx_data->control_port = EC_CONTROL;
	ctx_data->data_port = EC_DATA;

	if (!tuxec_read_reg(0xf9, &reg_value)) {
		msg_perr("Failed to query flash ROM size.\n");
		return false;
	}
	switch (reg_value & 0xf0) {
	case 0x40:
		ctx_data->rom_size_in_blocks = 3;
		break;
	case 0xf0:
		ctx_data->rom_size_in_blocks = 4;
		break;
	default:
		ctx_data->rom_size_in_blocks = 1;
		break;
	}

	if (!tuxec_read_reg(0x10, &reg_value)) {
		msg_perr("Failed to query AC adapter state.\n");
		return false;
	}
	ctx_data->ac_adapter_plugged = reg_value & 0x01;

	ctx_data->rom_size_in_kbytes =
		ctx_data->rom_size_in_blocks*KBYTES_PER_BLOCK;

	return true;
}

static int tuxec_probe(struct flashctx *flash)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	flash->chip->tested = TEST_OK_PREW;
	flash->chip->page_size = BYTES_PER_BLOCK;
	flash->chip->total_size = ctx_data->rom_size_in_kbytes;

	flash->chip->block_erasers[0].eraseblocks[0].size = 1024;
	flash->chip->block_erasers[0].eraseblocks[0].count =
		ctx_data->rom_size_in_kbytes;

	return 1;
}

static int tuxec_read(struct flashctx *flash, uint8_t *buf,
		      unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int from_byte = start - start%BYTES_PER_BLOCK;
	const unsigned int to_byte = ctx_data->rom_size_in_kbytes*1024;

	unsigned int offset;
	uint8_t rom_byte;

	int ret = 0;

	for (offset = from_byte; offset < to_byte; ++offset) {
		if (offset%BYTES_PER_BLOCK == 0) {
			if (!tuxec_write_cmd(ctx_data, EC_CMD_READ_BLOCK) ||
			    !tuxec_write_cmd(ctx_data, offset/BYTES_PER_BLOCK))
				ret = 1;
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
	while (tuxec_read_byte(ctx_data, &rom_byte))
		continue;

	return ret;
}

static bool tuxec_write_patched(tuxec_data_t *ctx_data, unsigned int offset,
				uint8_t data)
{
	if (ctx_data->autoload_state != AUTOLOAD_THREE_BYTES)
		return tuxec_write_byte(ctx_data, data);

	switch (ctx_data->autload_action) {
	case AUTOLOAD_NO_ACTION:
		return tuxec_write_byte(ctx_data, data);

	case AUTOLOAD_DISABLE:
		if (offset == ctx_data->autoload_offset + 2) {
			if (ctx_data->rom_size_in_blocks == 1 ||
			    ctx_data->rom_size_in_blocks == 2) {
				data = 0x94;
			} else {
				data = 0x85;
			}
		} else if (offset == ctx_data->autoload_offset + 8) {
			data = 0x00;
		}
		break;

	case AUTOLOAD_SETON:
		if (offset == ctx_data->autoload_offset + 2) {
			if (ctx_data->rom_size_in_blocks == 1 ||
			    ctx_data->rom_size_in_blocks == 2) {
				data = 0x94;
			} else {
				data = 0x85;
			}
		} else if (offset == ctx_data->autoload_offset + 8) {
			if (ctx_data->rom_size_in_blocks == 1 ||
			    ctx_data->rom_size_in_blocks == 2) {
				data = 0x7f;
			} else {
				data = 0xbe;
			}
		}
		break;

	case AUTOLOAD_SETOFF:
		if (offset == ctx_data->autoload_offset + 2) {
			if (ctx_data->rom_size_in_blocks == 1 ||
			    ctx_data->rom_size_in_blocks == 2) {
				data = 0xa5;
			} else {
				data = 0xb5;
			}
		} else if (offset == ctx_data->autoload_offset + 8) {
			data = 0xaa;
		}
		break;
	}

	return tuxec_write_byte(ctx_data, data);
}

static bool tuxec_write_block(tuxec_data_t *ctx_data, const uint8_t *buf,
			      unsigned int block)
{
	const unsigned int block_start = block*BYTES_PER_BLOCK;

	unsigned int i;

	bool ret = true;

	if (!tuxec_write_cmd(ctx_data, EC_CMD_WRITE_BLOCK) ||
	    !tuxec_write_cmd(ctx_data, 0x02) ||
	    !tuxec_write_cmd(ctx_data, block) ||
	    !tuxec_write_cmd(ctx_data, 0x02) ||
	    !tuxec_write_cmd(ctx_data, 0x02))
		ret = false;

	for (i = 0; i < BYTES_PER_BLOCK; ++i) {
		if (!tuxec_write_patched(ctx_data, block_start  + i, *buf))
			ret = false;
		++buf;
	}

	return ret;
}

static void tuxec_update_autoload_state(tuxec_data_t *ctx_data,
					const uint8_t *buf,
					unsigned int start, unsigned int len)
{
	unsigned int i;

	if (ctx_data->autload_action == AUTOLOAD_NO_ACTION ||
	    ctx_data->autoload_state == AUTOLOAD_THREE_BYTES)
		return;

	for (i = 0; i < len; ++i) {
		switch (ctx_data->autoload_state) {
		case AUTOLOAD_NONE:
			if (buf[i] == 0xa5) {
				ctx_data->autoload_state = AUTOLOAD_ONE_BYTE;
			}
			continue;
		case AUTOLOAD_ONE_BYTE:
			if (buf[i] == 0xa5 || buf[i] == 0xa4) {
				ctx_data->autoload_state = AUTOLOAD_TWO_BYTES;
			}
			continue;
		case AUTOLOAD_TWO_BYTES:
			if (buf[i] == 0x5a) {
				ctx_data->autoload_state = AUTOLOAD_THREE_BYTES;
				ctx_data->autoload_offset = start + i;
			}
			continue;
		case AUTOLOAD_THREE_BYTES:
			return;
		}

		ctx_data->autoload_state = AUTOLOAD_NONE;
	}
}

static int tuxec_write(struct flashctx *flash, const uint8_t *buf,
				unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int from_block = start/BYTES_PER_BLOCK;
	const unsigned int end = start + len;

	unsigned int block;
	unsigned int to_block;
	unsigned int offset = start;
	unsigned int left = len;

	to_block = end/BYTES_PER_BLOCK;
	if (end%BYTES_PER_BLOCK != 0)
		++to_block;

	tuxec_update_autoload_state(ctx_data, buf, start, len);

	for (block = from_block; block < to_block; ++block) {
		const unsigned int block_start = block*BYTES_PER_BLOCK;

		uint8_t *tmp_buf;
		unsigned int block_end;
		unsigned int update_size;

		if (offset == block_start && left >= BYTES_PER_BLOCK) {
			if (!tuxec_write_block(ctx_data, buf, block)) {
				msg_perr("Unable to write full block.\n");
				return 1;
			}

			offset += BYTES_PER_BLOCK;
			left -= BYTES_PER_BLOCK;
			continue;
		}

		block_end = block_start + BYTES_PER_BLOCK;
		update_size = min(left, block_end - offset);

		tmp_buf = (uint8_t *)malloc(BYTES_PER_BLOCK);
		if (!tmp_buf) {
			msg_perr("Unable to allocate space for block "
				 "buffer.\n");
			return 1;
		}

		if (read_opaque(flash, tmp_buf, block_start, BYTES_PER_BLOCK)) {
			free(tmp_buf);
			msg_perr("Unable to read block into buffer.\n");
			return 1;
		}

		memcpy(tmp_buf + (offset - block_start), buf, update_size);

		if (!tuxec_write_block(ctx_data, tmp_buf, block)) {
			free(tmp_buf);
			msg_perr("Unable to write updated block.\n");
			return 1;
		}

		free(tmp_buf);

		offset += update_size;
		left -= update_size;
	}

	return 0;
}

static int tuxec_erase(struct flashctx *flash,
		       unsigned int start, unsigned int len)
{
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	const unsigned int from_chunk = start/BYTES_PER_CHUNK;
	const unsigned int end = start + len;

	unsigned int to_chunk;

	unsigned int i;
	int ret = 0;

	to_chunk = end/BYTES_PER_CHUNK;
	if (end%BYTES_PER_CHUNK != 0)
		++to_chunk;

	for (i = from_chunk; i < to_chunk; i += CHUNKS_PER_KBYTE) {
		if (!tuxec_write_cmd(ctx_data, EC_CMD_ERASE_KBYTE) ||
		    !tuxec_write_cmd(ctx_data, i/CHUNKS_PER_BLOCK) ||
		    !tuxec_write_cmd(ctx_data, i%CHUNKS_PER_BLOCK) ||
		    !tuxec_write_cmd(ctx_data, 0x00))
			ret = 1;

		internal_sleep(1000);
	}

	internal_sleep(100000);
	return ret;
}

static struct opaque_master programmer_tuxec = {
	.max_data_read  = MAX_DATA_READ_UNLIMITED,
	.max_data_write = MAX_DATA_WRITE_UNLIMITED,
	.probe		= tuxec_probe,
	.read		= tuxec_read,
	.write		= tuxec_write,
	.erase		= tuxec_erase,
};

static bool tuxec_check_params(tuxec_data_t *ctx_data)
{
	char *p;

	bool ret = true;

	p = extract_programmer_param("type");
	if (p && strcmp(p, "ec")) {
		msg_pdbg("%s(): tuxec only supports \"ec\" type devices\n",
			 __func__);
		ret = false;
	}
	free(p);

	p = extract_programmer_param("noaccheck");
	if (p && strcmp(p, "yes") == 0) {
		/* Assume it's always present. */
		ctx_data->ac_adapter_plugged = true;
	}
	free(p);

	p = extract_programmer_param("portpair");
	if (p) {
		if (!strcmp(p, "0")) {
			ctx_data->control_port = 0x64;
			ctx_data->data_port = 0x60;
		} else if (!strcmp(p, "1")) {
			ctx_data->control_port = 0x66;
			ctx_data->data_port = 0x62;
		} else if (!strcmp(p, "2")) {
			ctx_data->control_port = 0x6c;
			ctx_data->data_port = 0x68;
		} else if (!strcmp(p, "3")) {
			ctx_data->control_port = 0x6e;
			ctx_data->data_port = 0x6a;
		} else {
			msg_pdbg("%s(): incorrect portpair param value: %s\n",
				__func__, p);
			ret = false;
		}
	}
	free(p);

	p = extract_programmer_param("autoload");
	if (p) {
		if (!strcmp(p, "none")) {
			ctx_data->autload_action = AUTOLOAD_NO_ACTION;
		} else if (!strcmp(p, "disable")) {
			ctx_data->autload_action = AUTOLOAD_DISABLE;
		} else if (!strcmp(p, "on")) {
			ctx_data->autload_action = AUTOLOAD_SETON;
		} else if (!strcmp(p, "off")) {
			ctx_data->autload_action = AUTOLOAD_SETOFF;
		} else {
			msg_pdbg("%s(): incorrect autoload param value: %s\n",
				__func__, p);
			ret = false;
		}
	}
	free(p);

	return ret;
}

int tuxec_init(void)
{
	bool read_success;
	uint8_t write_type;

	tuxec_data_t *ctx_data = NULL;

	msg_pdbg("%s(): entered\n", __func__);

	ctx_data = calloc(1, sizeof(tuxec_data_t));
	if (!ctx_data) {
		msg_perr("Unable to allocate space for extra context data.\n");
		return 1;
	}

	if (!tuxec_write_reg(0xf9, 0x20) ||
	    !tuxec_write_reg(0xfa, 0x02) ||
	    !tuxec_write_reg(0xfb, 0x00) ||
	    !tuxec_write_reg(0xf8, 0xb1)) {
		msg_perr("Unable to initialize controller.\n");
		goto init_err_exit;
	}

	if (!tuxec_init_ctx(ctx_data))
		goto init_err_exit;

	if (!tuxec_check_params(ctx_data))
		goto init_err_exit;


	if (!tuxec_write_cmd(ctx_data, 0xde) ||
	    !tuxec_write_cmd(ctx_data, 0xdc) ||
	    !tuxec_write_cmd(ctx_data, 0xf0)) {
		msg_perr("Failed to write identification commands.\n");
		goto init_err_exit;
	}

	read_success = tuxec_read_byte(ctx_data, &write_type);
	if (read_success && write_type != 0 && write_type != 0xff) {
		msg_perr("ITE5570 is not supported.\n");
		goto init_err_exit;
	}

	if (!ctx_data->ac_adapter_plugged) {
		msg_perr("AC adapter is not plugged.\n");
		goto init_err_exit;
	}

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
