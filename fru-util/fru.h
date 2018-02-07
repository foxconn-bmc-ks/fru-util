// Copyright (C) Microsoft Corporation. All rights reserved.
//
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "i2clib.h"

#ifndef _FRU_H
#define _FRU_H

/* fru definitions */
#define FRU_LANG			0x00
#define FRU_VERSION			0x01
#define FRU_LENGTH_MASK		0x3F
#define FRU_AREA_STOP		0xC1

#define I2C_BUS_0			0x04
#define I2C_BUS_1			0x01

/* eeprom addresses */
#define	EEPROM_RMB_ADDRESS	0xA0
#define	EEPROM_PMDU_ADDRESS	0xA2
#define	EEPROM_ROW_ADDRESS	0xA4
#define	EEPROM_AUX_ADDRESS	0xA0

/* eeprom limits */
#define MAX_RECORDS			18
#define MAX_NAME_LEN		18
#define MAX_LENGTH			62
#define MAX_EEPROM_SZ		1280

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
/*#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )*/

typedef enum {
	EM_FRU_EEPROM_AT24C64 = 0,
	EM_FRU_EEPROM_AT24C02,
	EM_FRU_EEPROM_PRODUCT_TYPE_MAX
} FRU_EEPROM_PRODUCT_TYPE;

/* fru eeprom header format */
PACK(typedef struct fru_header
{
	uint8_t        commonheader;
	uint8_t        areaoffset;
	uint8_t        chassis;
	uint8_t        board;
	uint8_t        product;
	uint8_t        multirecord;
	uint8_t        pad;
	uint8_t		   checksum;
}) FRU_HEADER;

typedef enum {
	EM_MSG_TYPE_HEX = 0,
	EM_MSG_TYPE_STRING,
	EM_MSG_TYPE_DATE_STRING,
} FRU_INFO_SHOW_MSG_TYPE;

/* fru area field */
PACK(typedef struct fru_area_info_field
{
	const char  *fru_field_name;
	uint8_t		*data;
	int 		length;
	FRU_INFO_SHOW_MSG_TYPE em_show_msg_type;
}) FRU_AREA_INFO_FIELD;

typedef enum {
	EM_CHASSIS = 0,
	EM_BOARD,
	EM_PRODUCT,
	EM_INTERNAL_USE,
	EM_FRU_INFO_MAX
} FRU_INFO_TYPE;

/* fru area field */
PACK(typedef struct fru_area_info_header
{
	FRU_INFO_TYPE em_fru_type;
	FRU_AREA_INFO_FIELD *ptr_fru_field;
	//(header_offset+notify_total_field_length_offset) mean
	// absolute offset in memory which sets area length
	uint8_t		notify_area_length_offset;
	uint8_t		area_length_multiply;
	uint8_t		header_offset;
	uint8_t		*area_data;
	uint16_t	area_length;
}) FRU_AREA_INFO_HEADER;

#endif
