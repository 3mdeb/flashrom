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
#include "ec.h"
#include "hwaccess.h"
#include "programmer.h"

#define EC_CMD_ERASE_ALL    0x01
#define EC_CMD_WRITE_BLOCK  0x02
#define EC_CMD_READ_BLOCK   0x03
#define EC_CMD_GET_FLASH_ID 0x04
#define EC_CMD_ERASE_KBYTE  0x05
#define EC_CMD_WRITE_KBYTE  0x06
#define EC_CMD_READ_PRJ     0x92
#define EC_CMD_READ_VER     0x93

#define BYTES_PER_BLOCK     (64 * 1024)
#define BYTES_PER_CHUNK     256
#define KBYTES_PER_BLOCK    64
#define CHUNKS_PER_KBYTE    4
#define CHUNKS_PER_BLOCK    256

#define INFO_BUFFER_SIZE    16

enum autoloadaction {
	AUTOLOAD_NO_ACTION,
	AUTOLOAD_DISABLE,
	AUTOLOAD_SETON,
	AUTOLOAD_SETOFF
};

/* Need to match a pattern that spans 6 bytes to find patching place */
enum autoloadseekstate {
	AUTOLOAD_NONE,
	AUTOLOAD_ONE_BYTE,
	AUTOLOAD_TWO_BYTES,
	AUTOLOAD_THREE_BYTES,
	AUTOLOAD_FOUR_BYTES,
	AUTOLOAD_FIVE_BYTES,
	AUTOLOAD_SIX_BYTES
};

typedef struct
{
	unsigned int rom_size_in_blocks;
	unsigned int rom_size_in_kbytes;

	unsigned int autoload_offset; /* meaningful for AUTOLOAD_SIX_BYTES */
	enum autoloadseekstate autoload_state;
	enum autoloadaction autload_action;

	uint8_t *first_kbyte;
	bool support_ite5570;
	uint8_t write_mode;

	uint8_t control_port;
	uint8_t data_port;
	bool ac_adapter_plugged;
} tuxec_data_t;

static bool tuxec_write_cmd(tuxec_data_t *ctx_data, uint8_t cmd)
{
	return ec_write_cmd(ctx_data->control_port, cmd);
}

static bool tuxec_read_byte(tuxec_data_t *ctx_data, uint8_t *data)
{
	return ec_read_byte(ctx_data->control_port, ctx_data->data_port, data);
}

static bool tuxec_write_byte(tuxec_data_t *ctx_data, uint8_t data)
{
	return ec_write_byte(ctx_data->control_port, ctx_data->data_port, data);
}

static int tuxec_shutdown(void *data)
{
	msg_pdbg("%s \n", __func__);
	tuxec_data_t *ctx_data = (tuxec_data_t *)data;

	/* In case we allocated, but didn't use the stash. */
	if (ctx_data->first_kbyte) {
		free(ctx_data->first_kbyte);
	}

	if (!tuxec_write_cmd(ctx_data, 0xfe))
		msg_pdbg("%s(): failed to finalize controller\n", __func__);

	free(data);
	return 0;
}

static void tuxec_read_project(tuxec_data_t *ctx_data)
{
	uint8_t ec_project[INFO_BUFFER_SIZE];
	uint8_t i;

	// dummy read
	tuxec_read_byte(ctx_data, NULL);

	if (!tuxec_write_cmd(ctx_data, EC_CMD_READ_PRJ)) {
		msg_perr("Failed to write cmd...\n");
		return;
	}

	for (i = 0; i < INFO_BUFFER_SIZE -1; ++i) {
		if (!tuxec_read_byte(ctx_data, &ec_project[i])) {
			msg_perr("Failed to read EC project...\n");
			return;
		}
		if (ec_project[i] == '$') {
			break;
		}
	}
	ec_project[i] = '\0';

	msg_cinfo("Mainboard EC Project: %s\n", (char *)ec_project);
}

static void tuxec_read_version(tuxec_data_t *ctx_data)
{
	uint8_t ec_version[INFO_BUFFER_SIZE];
	uint8_t i;

	// dummy read
	tuxec_read_byte(ctx_data, NULL);

	if (!tuxec_write_cmd(ctx_data, EC_CMD_READ_VER)) {
		msg_perr("Failed to write cmd...\n");
		return;
	}

	for (i = 0; i < INFO_BUFFER_SIZE -1; ++i) {
		if (!tuxec_read_byte(ctx_data, &ec_version[i])) {
			msg_perr("Failed to read EC version...\n");
			return;
		}
		if (ec_version[i] == '$') {
			break;
		}
	}
	ec_version[i] = '\0';

	msg_cinfo("Mainboard EC Version: %s\n", (char *)ec_version);
}

static bool tuxec_init_ctx(tuxec_data_t *ctx_data)
{
	msg_pdbg("%s \n", __func__);
	uint8_t reg_value;

	ctx_data->control_port = EC_CONTROL;
	ctx_data->data_port = EC_DATA;

	if (!ec_read_reg(0xf9, &reg_value)) {
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
		ctx_data->rom_size_in_blocks = 2;
		break;
	}

	// flush the EC registers
	INB(EC_CONTROL);
	INB(EC_DATA);

	if (!ec_read_reg(0x10, &reg_value)) {
		msg_perr("Failed to query first byte of state register.\n");
		return false;
	}
	if (!ec_read_reg(0x10, &reg_value)) {
		msg_perr("Failed to query AC adapter state.\n");
		return false;
	}
	ctx_data->ac_adapter_plugged = reg_value & 0x01;

	ctx_data->rom_size_in_kbytes =
		ctx_data->rom_size_in_blocks * KBYTES_PER_BLOCK;

	tuxec_read_project(ctx_data);
	tuxec_read_version(ctx_data);

	return true;
}

static int tuxec_read(struct flashctx *flash, uint8_t *buf,
		      unsigned int start, unsigned int len)
{
	unsigned int offset, block_index, block_start, block_end;
	uint8_t rom_byte;
	int ret = 0;
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	msg_pdbg("%s(): read flash @ 0x%x len %x\n", __func__, start, len);

	/*
	 * This EC can read only a whole block. So read whole block and return
	 * only the data that has been requested with start and len parameters.
	 */
	if (start > BYTES_PER_BLOCK)
		block_start = start / BYTES_PER_BLOCK;
	else
		block_start = 0;

	if ((start + len) > BYTES_PER_BLOCK) {
		if ((start + len) % BYTES_PER_BLOCK == 0) {
			block_end = (start + len) / BYTES_PER_BLOCK;
		} else {
			block_end = ((start + len) / BYTES_PER_BLOCK) + 1;
		}
	} else {
		block_end = 1;
	}

	if (block_end > ctx_data->rom_size_in_blocks){
		return 1;
	}

	for (block_index = block_start;
	     block_index < ctx_data->rom_size_in_blocks; block_index++) {
		if (!tuxec_write_cmd(ctx_data, EC_CMD_READ_BLOCK) ||
		    !tuxec_write_cmd(ctx_data, block_index)) {
			msg_perr("Failed to select block to read %d\n",
				 block_index);
			return 1;
		}
		for (offset = 0; offset < BYTES_PER_BLOCK; ++offset) {

			if (!tuxec_read_byte(ctx_data, &rom_byte)) {
				msg_perr("Flash read failed @ 0x%x\n", offset);
				return 1;
			}

			if ((block_index * BYTES_PER_BLOCK) + offset < start) {
				continue;
			}

			*buf = rom_byte;
			++buf;
			--len;

			if (len == 0) {
				break;
			}
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
	const bool blocks_1_2 = ctx_data->rom_size_in_blocks == 1 ||
				ctx_data->rom_size_in_blocks == 2;

	if (ctx_data->autoload_state != AUTOLOAD_SIX_BYTES)
		return tuxec_write_byte(ctx_data, data);

	switch (ctx_data->autload_action) {
	case AUTOLOAD_NO_ACTION:
		return tuxec_write_byte(ctx_data, data);
	case AUTOLOAD_DISABLE:
		if (offset == ctx_data->autoload_offset + 2) {
			data = (blocks_1_2 ? 0x94 : 0x85);
		} else if (offset == ctx_data->autoload_offset + 8) {
			data = 0x00;
		}
		break;
	case AUTOLOAD_SETON:
		if (offset == ctx_data->autoload_offset + 2) {
			data = (blocks_1_2 ? 0x94 : 0x85);
		} else if (offset == ctx_data->autoload_offset + 8) {
			data = (blocks_1_2 ? 0x7f : 0xbe);
		}
		break;
	case AUTOLOAD_SETOFF:
		if (offset == ctx_data->autoload_offset + 2) {
			data = (blocks_1_2 ? 0xa5 : 0xb5);
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
	const unsigned int param = ctx_data->support_ite5570 ? 0x00 : 0x02;
	unsigned int offset = block * BYTES_PER_BLOCK;
	unsigned int third_param = param;
	unsigned int i;
	bool ret = true;

	msg_pdbg("%s(): write flash block %d\n", __func__, block);

	/* Stash first kilobyte and write it after the last block. */
	if (ctx_data->write_mode != 0 && block == 0) {
		ctx_data->first_kbyte = (uint8_t *)malloc(1024);
		if (!ctx_data->first_kbyte) {
			msg_perr("Failed to allocate memory for a stash.\n");
			return false;
		}

		memcpy(ctx_data->first_kbyte, buf, 1024);

		third_param = 0x04;
	}


	if (!tuxec_write_cmd(ctx_data, EC_CMD_WRITE_BLOCK) ||
	    !tuxec_write_cmd(ctx_data, param) ||
	    !tuxec_write_cmd(ctx_data, block) ||
	    !tuxec_write_cmd(ctx_data, third_param) ||
	    !tuxec_write_cmd(ctx_data, param))
		ret = false;

	for (i = 0; i < BYTES_PER_BLOCK; ++i, ++offset) {
		if (!tuxec_write_patched(ctx_data, offset, buf[i]))
			ret = false;
	}

	/* If we're done, write the first kilobyte separately. */
	if (ctx_data->write_mode != 0 &&
	    block == ctx_data->rom_size_in_blocks - 1) {
		if (!ctx_data->first_kbyte) {
			msg_perr("No first KB stash was found.\n");
			return false;
		}

		if (!tuxec_write_cmd(ctx_data, 0x06))
			ret = false;

		for (i = 0; i < 1024; ++i) {
			if (!tuxec_write_patched(ctx_data, i,
						 ctx_data->first_kbyte[i]))
				ret = false;
		}

		free(ctx_data->first_kbyte);
		ctx_data->first_kbyte = NULL;
	}

	return ret;
}

static void tuxec_update_autoload_state(tuxec_data_t *ctx_data,
					const uint8_t *buf,
					unsigned int start, unsigned int len)
{
	msg_pdbg("%s \n", __func__);
	unsigned int i;

	unsigned int offset = start;

	if (ctx_data->autload_action == AUTOLOAD_NO_ACTION ||
	    ctx_data->autoload_state == AUTOLOAD_SIX_BYTES)
		return;

	for (i = 0; i < len; ++i, ++offset) {
		switch (ctx_data->autoload_state) {
		case AUTOLOAD_NONE:
			if (buf[i] != 0xa5)
				break;
			ctx_data->autoload_state = AUTOLOAD_ONE_BYTE;
			ctx_data->autoload_offset = offset;
			continue;
		case AUTOLOAD_ONE_BYTE:
			if (offset - ctx_data->autoload_offset != 1)
				break;
			if (buf[i] != 0xa5 && buf[i] != 0xa4)
				break;
			ctx_data->autoload_state = AUTOLOAD_TWO_BYTES;
			continue;
		case AUTOLOAD_TWO_BYTES:
			if (offset - ctx_data->autoload_offset != 2)
				break;
			ctx_data->autoload_state = AUTOLOAD_THREE_BYTES;
			continue;
		case AUTOLOAD_THREE_BYTES:
			if (offset - ctx_data->autoload_offset != 3)
				break;
			ctx_data->autoload_state = AUTOLOAD_FOUR_BYTES;
			continue;
		case AUTOLOAD_FOUR_BYTES:
			if (offset - ctx_data->autoload_offset != 4)
				break;
			ctx_data->autoload_state = AUTOLOAD_FIVE_BYTES;
			continue;
		case AUTOLOAD_FIVE_BYTES:
			if (offset - ctx_data->autoload_offset != 5)
				break;
			if (buf[i] != 0x5a)
				break;
			ctx_data->autoload_state = AUTOLOAD_SIX_BYTES;
			continue;
		case AUTOLOAD_SIX_BYTES:
			/* Done matching. */
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

	msg_pdbg("\n%s(): write flash @ 0x%x len %x\n", __func__, start, len);

	tuxec_update_autoload_state(ctx_data, buf, start, len);

	for (block = from_block; block < to_block; ++block) {
		const unsigned int block_start = block * BYTES_PER_BLOCK;
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

		offset += update_size;
		left -= update_size;
	}

	return 0;
}

static int tuxec_full_erase(tuxec_data_t *ctx_data)
{
	unsigned int i;

	msg_pdbg("\n%s(): full erase flash \n", __func__);

	if (!tuxec_write_cmd(ctx_data, EC_CMD_ERASE_ALL) ||
	    !tuxec_write_cmd(ctx_data, 0x00) ||
	    !tuxec_write_cmd(ctx_data, 0x00) ||
	    !tuxec_write_cmd(ctx_data, 0x00) ||
	    !tuxec_write_cmd(ctx_data, 0x00))
		return 1;

	if (ctx_data->rom_size_in_blocks < 3) {
		internal_sleep(15000 * 64);
		return 0;
	}

	for (i = 0; i < 4; ++i) {
		if (!ec_wait_for_obuf(ctx_data->control_port,
				      EC_MAX_STATUS_CHECKS * 3))
			return 1;

		if (INB(ctx_data->data_port) == 0xf8)
			return 0;
	}
	return 1;
}

static int tuxec_chunkwise_erase(tuxec_data_t *ctx_data,
				 unsigned int start, unsigned int len)
{
	const unsigned int from_chunk = start / BYTES_PER_CHUNK;
	const unsigned int end = start + len;
	unsigned int to_chunk;
	unsigned int i;
	int ret = 0;

	msg_pdbg("\n%s(): erase flash @ 0x%x len %x\n", __func__, start, len);

	to_chunk = end / BYTES_PER_CHUNK;
	if (end % BYTES_PER_CHUNK != 0)
		++to_chunk;

	for (i = from_chunk; i < to_chunk; i += CHUNKS_PER_KBYTE) {
		if (!tuxec_write_cmd(ctx_data, EC_CMD_ERASE_KBYTE) ||
		    !tuxec_write_cmd(ctx_data, i / CHUNKS_PER_BLOCK) ||
		    !tuxec_write_cmd(ctx_data, i % CHUNKS_PER_BLOCK) ||
		    !tuxec_write_cmd(ctx_data, 0x00))
			return 1;

		internal_sleep(1000);
	}

	internal_sleep(100000);
	return ret;
}

static int tuxec_erase(struct flashctx *flash, unsigned int blockaddr,
		       unsigned int blocklen)
{
	msg_pdbg("%s \n", __func__);
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	if (ctx_data->support_ite5570)
		return tuxec_chunkwise_erase(ctx_data, blockaddr, blocklen);

	return tuxec_full_erase(ctx_data);
}


static int tuxec_probe(struct flashctx *flash)
{
	msg_pdbg("%s \n", __func__);
	tuxec_data_t *ctx_data = (tuxec_data_t *)flash->mst->opaque.data;

	flash->chip->tested = TEST_OK_PREW;
	flash->chip->page_size = BYTES_PER_BLOCK;
	flash->chip->total_size = ctx_data->rom_size_in_kbytes;

	if (ctx_data->support_ite5570) {
		flash->chip->block_erasers[0].eraseblocks[0].size = 1024;
		flash->chip->block_erasers[0].eraseblocks[0].count =
			ctx_data->rom_size_in_kbytes;
	} else {
		flash->chip->block_erasers[0].eraseblocks[0].size =
			ctx_data->rom_size_in_blocks * BYTES_PER_BLOCK;
		flash->chip->block_erasers[0].eraseblocks[0].count = 1;
	}

	flash->chip->block_erasers[0].block_erase = tuxec_erase;

	flash->chip->gran = write_gran_64kbytes;

	return 1;
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
	msg_pdbg("%s \n", __func__);
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
		/* Just mark it as present. */
		ctx_data->ac_adapter_plugged = true;
	}
	free(p);

	p = extract_programmer_param("ite5570");
	if (p && strcmp(p, "yes") == 0) {
		ctx_data->support_ite5570 = true;
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

	p = extract_programmer_param("romsize");
	if (p) {
		if (!strcmp(p, "64K")) {
			ctx_data->rom_size_in_blocks = 1;
		} else if (!strcmp(p, "128K")) {
			ctx_data->rom_size_in_blocks = 2;
		} else if (!strcmp(p, "192K")) {
			ctx_data->rom_size_in_blocks = 3;
		} else if (!strcmp(p, "256K")) {
			ctx_data->rom_size_in_blocks = 4;
		} else {
			msg_pdbg("%s(): incorrect romsize param value: %s\n",
				__func__, p);
			ret = false;
		}

		ctx_data->rom_size_in_kbytes =
			ctx_data->rom_size_in_blocks * KBYTES_PER_BLOCK;
	}
	free(p);

	return ret;
}

static void tuxec_read_flash_id(tuxec_data_t *ctx_data)
{

	uint8_t rom_data;
	unsigned int id_length, i;

	msg_cinfo("%s(): Get Flash Part ID\n", __func__);
	tuxec_write_cmd (ctx_data, EC_CMD_GET_FLASH_ID);

	id_length = 3;
	if (ctx_data->rom_size_in_blocks == 3 ||
	    ctx_data->rom_size_in_blocks == 4)
		id_length = 4;

	msg_cinfo("Flash Part ID: ");
	for (i = 0; i < id_length; i++) {
		tuxec_read_byte (ctx_data, &rom_data);
		msg_cinfo("%02x ", rom_data);
	}

	msg_cinfo("\n");
}

int tuxec_init(void)
{
	msg_pdbg("%s \n", __func__);
	bool read_success;

	tuxec_data_t *ctx_data = NULL;

	msg_pdbg("%s(): entered\n", __func__);

	if (rget_io_perms())
		return 1;

	if (!ec_write_reg(0xf9, 0x20) ||
	    !ec_write_reg(0xfa, 0x02) ||
	    !ec_write_reg(0xfb, 0x00) ||
	    !ec_write_reg(0xf8, 0xb1)) {
		msg_perr("Unable to initialize controller.\n");
		return 1;
	}

	ctx_data = calloc(1, sizeof(tuxec_data_t));
	if (!ctx_data) {
		msg_perr("Unable to allocate space for extra context data.\n");
		return 1;
	}

	if (!tuxec_init_ctx(ctx_data))
		goto tuxec_init_exit;

	if (!tuxec_check_params(ctx_data))
		goto tuxec_init_exit;

	if (!tuxec_write_cmd(ctx_data, 0xde) ||
		!tuxec_write_cmd(ctx_data, 0xdc)) {
		msg_perr("%s(): failed to prepare controller\n", __func__);
		goto tuxec_init_exit;
	}

	if (!tuxec_write_cmd(ctx_data, 0xf0)) {
		msg_perr("Failed to write identification commands.\n");
		goto tuxec_init_exit_shutdown;
	}

	read_success = tuxec_read_byte(ctx_data, &ctx_data->write_mode);
	msg_pdbg("%s(): write mode %02x\n", __func__, ctx_data->write_mode);
	if (read_success && ctx_data->write_mode != 0x00 &&
	    ctx_data->write_mode != 0xff) {
		msg_pdbg("%s(): selecting ITE5570 support\n", __func__);
		ctx_data->support_ite5570 = true;
	} else {
		ctx_data->write_mode = 0;
	}

	tuxec_read_flash_id(ctx_data);

	if (!ctx_data->ac_adapter_plugged) {
		msg_perr("AC adapter is not plugged.\n");
		goto tuxec_init_exit_shutdown;
	}

	programmer_tuxec.data = ctx_data;

	if (register_shutdown(tuxec_shutdown, ctx_data))
		goto tuxec_init_exit_shutdown;
	if (register_opaque_master(&programmer_tuxec))
		return 1;

	msg_pdbg("%s(): successfully initialized tuxec\n", __func__);
	return 0;

tuxec_init_exit:
	tuxec_shutdown(ctx_data);
	return 1;

tuxec_init_exit_shutdown:
	tuxec_shutdown(ctx_data);
	return 1;
}

#endif
