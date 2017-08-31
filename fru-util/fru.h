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
#define MAX_NAME_LEN		128
#define MAX_LENGTH			256
#define MAX_EEPROM_SZ		768

/* Fru util */
#define CHASSIS_INFO_COUNT	14
#define BOARD_INFO_COUNT	19
#define PRODUCT_INFO_COUNT	25

#define CHASSIS_INFO_PROP	2
/* eeprom base addresses */
#define	EEPROM_CHASSIS_ADDRESS	0x80
#define	EEPROM_BOARD_ADDRESS	0x100
#define	EEPROM_PRODUCT_ADDRESS	0x180
#define	EEPROM_MULTIRECORD_ADDRESS	0x200

/* Info area size */
#define INFO_AREA_SIZE 0x7F

/* Operations */
#define OP_READ		0
#define OP_WRITE	1
#define OP_PATCH	2

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
/*#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )*/

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

/* fru area common header */
PACK(typedef struct area_header
{
	uint8_t        version : 4;
	uint8_t        length;
	uint8_t        languagecode;
}) AREA_HEADER;

/* fru area field */
PACK(typedef struct fru_field
{
	uint8_t			*length;
	uint8_t         *data;
}) AREA_FIELD;

/* fru board info area */
PACK(typedef struct fru_board_info
{
	AREA_HEADER 	header;
	uint8_t			mfgdatetime[3];
	AREA_FIELD      manufacture;
	AREA_FIELD      name;
	AREA_FIELD		serial;
	AREA_FIELD		part;
	AREA_FIELD		fruid;
	AREA_FIELD		address1;
	AREA_FIELD		address2;
	AREA_FIELD		boardver;
	AREA_FIELD		build;
}) FRU_BOARD_INFO;

/* fru product info area */
PACK(typedef struct fru_product_info
{
	AREA_HEADER		header;
	AREA_FIELD      manufacture;
	AREA_FIELD      productname;
	AREA_FIELD		productversion;
	AREA_FIELD		serial;
	AREA_FIELD		assettag;
	AREA_FIELD		fruid;
	AREA_FIELD		subproduct;
	AREA_FIELD		build;
}) FRU_PRODUCT_INFO;
