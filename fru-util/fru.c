// Copyright (C) Microsoft Corporation. All rights reserved.
//
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "fru_sup.h"
#include "ocslog.h"

/*#define DEBUG*/

/* end move to i2c library */
static int read_from_eeprom(uint8_t channel, uint8_t slave_addr);
static int write_to_eeprom(uint8_t channel, uint8_t slave_addr, uint16_t fru_offset, uint16_t write_length, uint8_t* buffer);
static int parse_and_check_fru_area_info(FRU_INFO_TYPE em_fru_type);

/* debug buffer */
uint8_t *debug_buffer;

/* function pointer for i2c write read */
int(*i2c_write_read)(int32_t handle,
	uint8_t slave_addr,
	uint8_t write_length,
	uint8_t* write_data,
	uint16_t read_length,
	uint8_t* buffer);

/* function pointer for i2c write */
int(*i2c_write)(int32_t handle,
	uint8_t slave_addr,
	uint8_t write_length,
	uint8_t* write_data,
	uint8_t data_length,
	uint8_t* data);

/*
	Required FRU fields.  These tags must appear in the FRU file.
*/
#define DEFAULT_FRU_FIELD_LENGTH (1)
#define FRU_FIELD_LENGTH_REFER_PREVIOUS (-1)
#define FRU_FIELD_LENGTH_REMAIN_SPACE (-2)
#define FRU_HEADER_OFFSET_MULTIPLY (8)
#define FRU_AREA_INFO_TYPE_MASK ((1<<7) | (1<<6))
#define MAX_FILE_PATH_LENGTH (200)

static FRU_AREA_INFO_FIELD CHASSIS_FRU_FIELDS[] = {
//      fru_field_name                        data  length                           em_show_msg_type
//      ---------------                       ----- --------                         -----------------------
        {"Version",                           NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Area length",                       NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Type",                      NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Part Number Type/Length",   NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Part Number String",        NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Chassis Serial Number Type/Length", NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Serial Number String",      NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Chassis Extra 1 UUID Type/Length",  NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Extra 1 UUID String",       NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Custom information Type/Length",    NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Chassis Custom information",        NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"End of Field Marker",               NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"PAD",                               NULL, FRU_FIELD_LENGTH_REMAIN_SPACE,   EM_MSG_TYPE_HEX},
        {"Checksum",                          NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {0},//End Array
};

static FRU_AREA_INFO_FIELD BOARD_FRU_FIELDS[] = {
//      fru_field_name                        data  length                           em_show_msg_type
//      ---------------                       ----- --------                         -----------------------
        {"Version",                           NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Area length",                       NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Info Language Code",          NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Manufacturing Date/Time",     NULL, 3,                               EM_MSG_TYPE_DATE_STRING},
        {"Board Manufacturer Type/Length",    NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Manufacturer String",         NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Board Product Name Type/Length",    NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Product Name String",         NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Board Serial Number Type/Length",   NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Serial Number String",        NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Board Part Number Type/Length",     NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Part Number String",          NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"FRU File ID Type/Length",           NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board FRU File ID String",          NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Custom information Type/Length",    NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Board Custom information",          NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"End of Field Marker",               NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"PAD",                               NULL, FRU_FIELD_LENGTH_REMAIN_SPACE,   EM_MSG_TYPE_HEX},
        {"Checksum",                          NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {0},//End Array
};

static FRU_AREA_INFO_FIELD PRODUCT_FRU_FIELDS[] = {
//      fru_field_name                        data  length                           em_show_msg_type
//      ---------------                       ----- --------                         -----------------------
        {"Version",                           NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Area length",                       NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Info Language code",        NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Manufacturer Name Type/Length",     NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Manufacturer Name String",  NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Product Name Type/Length",          NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Product Name String",       NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Part/Model Number Type/Length",     NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Part/Model Number String",  NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Product Version Type/Length",       NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Version String",            NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Product Serial Number Type/Length", NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Serial Number String",      NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Asset Tag Type/Length",             NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Asset Tag String",          NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"FRU File ID Type/Length",           NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product FRU File ID String",        NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Product Extra Rack ID Type/Length", NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Extra Rack ID String",      NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Product Extra Node ID Type/Length", NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Extra Node ID String",      NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"Custom information Type/Length",    NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Product Custom information",        NULL, FRU_FIELD_LENGTH_REFER_PREVIOUS, EM_MSG_TYPE_STRING},
        {"End of Field Marker",               NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"PAD",                               NULL, FRU_FIELD_LENGTH_REMAIN_SPACE,   EM_MSG_TYPE_HEX},
        {"Checksum",                          NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {0},//End Array
};

static FRU_AREA_INFO_FIELD INTERNAL_USE_AREA_FRU_FIELDS[] = {
//      fru_field_name                        data  length                           em_show_msg_type
//      ---------------                       ----- --------                         -----------------------
        {"Internal Use Area Format Version",  NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Type",                              NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Length",                            NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte1",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte2",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte3",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte4",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte5",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"MAC Address Byte6",                 NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"Number of MAC",                     NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {"PAD",                               NULL, FRU_FIELD_LENGTH_REMAIN_SPACE,   EM_MSG_TYPE_HEX},
        {"Internal Use Area checksum",        NULL, DEFAULT_FRU_FIELD_LENGTH,        EM_MSG_TYPE_HEX},
        {0},//End Array
};

static FRU_AREA_INFO_HEADER FRU_AREA_HEADER[] = {
//       em_fru_type      ptr_fru_field                notify_area_length_offset  area_length_multiply  header_offset    area_data    area_length
//       ------------     ---------------              ---------------            --------------        --------------   ----------   ------------
        {EM_CHASSIS,      CHASSIS_FRU_FIELDS,           1,                        8,                    0,               NULL,        0},
        {EM_BOARD,        BOARD_FRU_FIELDS,             1,                        8,                    0,               NULL,        0},
        {EM_PRODUCT,      PRODUCT_FRU_FIELDS,           1,                        8,                    0,               NULL,        0},
        {EM_INTERNAL_USE, INTERNAL_USE_AREA_FRU_FIELDS, 2,                        1,                    0,               NULL,        0},
};

/*
	assigns fru charactor data in buffer to fru structures.
*/
static int read_fru_from_buffer(uint8_t *buffer, uint16_t length, FRU_INFO_TYPE em_fru_type)
{
	uint16_t absolute_area_header_offset = 0;
	uint16_t absolute_area_length_offset = 0;
	uint8_t  area_length;

	if (em_fru_type >= EM_FRU_INFO_MAX) {
		log_fnc_err(UNKNOWN_ERROR, "read_fru_from_buffer error :em_fru_type:%d \n", em_fru_type);
		return FAILURE;
	}
	//it has no need to read fru area while header offset is zero or fru_field is undefined
	if (FRU_AREA_HEADER[em_fru_type].header_offset == 0 || FRU_AREA_HEADER[em_fru_type].ptr_fru_field == NULL)
		return UNKNOWN_ERROR;

	absolute_area_header_offset = (FRU_AREA_HEADER[em_fru_type].header_offset * FRU_HEADER_OFFSET_MULTIPLY);
	absolute_area_length_offset =  absolute_area_header_offset + FRU_AREA_HEADER[em_fru_type].notify_area_length_offset;
	if (absolute_area_length_offset >= length) {
		log_fnc_err(UNKNOWN_ERROR, "read_fru_from_buffer error :offset exceed buffer length:%d, %d \n", absolute_area_length_offset, length);
		return FAILURE;
	}
	area_length = buffer[absolute_area_length_offset];
	if (area_length <= 0)
		return UNKNOWN_ERROR;
	FRU_AREA_HEADER[em_fru_type].area_length = area_length * FRU_AREA_HEADER[em_fru_type].area_length_multiply;
	FRU_AREA_HEADER[em_fru_type].area_data = &(buffer[absolute_area_header_offset]);
	if ((FRU_AREA_HEADER[em_fru_type].area_length + absolute_area_header_offset) >= length) {
		log_fnc_err(UNKNOWN_ERROR, "read_fru_from_buffer error :fru data size exceed buffer length:%d, %d \n",
				(FRU_AREA_HEADER[em_fru_type].area_length + absolute_area_header_offset), length);
		return FAILURE;
	}
	return parse_and_check_fru_area_info(em_fru_type);
}

/* debug simulating i2c_write_read */
static int i2c_write_read_dbg(int32_t handle, uint8_t slave_addr,
	uint8_t write_length, uint8_t* write_data, uint16_t read_length, uint8_t* buffer)
{
	if (debug_buffer == NULL){
		log_fnc_err(UNKNOWN_ERROR, "i2c_write_read_dbg - no debug buffer defined");
		return FAILURE;
	}
	else{
		uint16_t offset = 0;
		memcpy(&offset, write_data, sizeof(uint16_t));
		memcpy(buffer, &debug_buffer[offset], read_length);
		return SUCCESS;
	}
}

/* write read from i2c device */
static int i2c_write_read_prod(int32_t handle, uint8_t slave_addr,
	uint8_t write_length, uint8_t* write_data, uint16_t read_length, uint8_t* buffer)
{
	// switch msb->lsb
	uint16_t offset = (uint16_t)(write_data[0]<<8|write_data[1]);
	memcpy(write_data, &offset, sizeof(uint16_t));

	if(i2c_block_read(handle, slave_addr, write_length, write_data, read_length, buffer)!= SUCCESS){
		log_fnc_err(UNKNOWN_ERROR, "i2c read_after_write failed for eeprom: (%02x).\n", slave_addr);
		return FAILURE;
	}

	return SUCCESS;
}

/* debug simulating i2c_write */
static int i2c_write_dbg(int32_t handle, uint8_t slave_addr,
	uint8_t write_length, uint8_t* write_data, uint8_t data_length, uint8_t* data)
{
	if (debug_buffer == NULL){
		log_out("i2c_write_read_dbg - no debug buffer defined");
		return FAILURE;
	}
	else{
		uint16_t offset = 0;
		memcpy(&offset, write_data, sizeof(uint16_t));
		memcpy(&debug_buffer[offset], data, data_length);
		return SUCCESS;
	}
}

/* write to i2c device */
static int i2c_write_prod(int32_t handle, uint8_t slave_addr,
	uint8_t write_length, uint8_t *write_data, uint8_t data_length, uint8_t *buffer)
{
		uint16_t offset = (uint16_t)(write_data[0]<<8 | write_data[1]);
		memcpy(write_data, &offset, sizeof(uint16_t));
		if(i2c_block_write(handle, slave_addr, write_length, write_data, data_length, buffer) != SUCCESS)
		{
				log_fnc_err(UNKNOWN_ERROR, "i2c write failed for eeprom (%02x).\n", slave_addr);
				return FAILURE;
		}

		return SUCCESS;
}

/* reads raw fru data from eeprom */
static int read_raw_from_eeprom(uint8_t channel, uint8_t slave_addr) {

	print_msg("reading raw from eeprom", NULL);

	uint8_t *buffer;
	buffer = calloc(MAX_EEPROM_SZ, sizeof(uint8_t));

	uint16_t fru_offset = 0;
	uint16_t buf_idx = 0;
	uint8_t  write_buffer[sizeof(uint16_t)];

	int response = 0;
	uint8_t read_length = 0;
	// copy the start offset to the write buffer
	memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

	int32_t handle = 0;
	// open i2c bus
	if ((response = open_i2c_channel(channel, &handle)) != SUCCESS)
		log_fnc_err(UNKNOWN_ERROR, "unable to open i2c bus");

	log_out("\n");
	if (response == SUCCESS) {
		while (fru_offset < MAX_EEPROM_SZ)
		{
			// copy the start offset to the write buffer
			memset(write_buffer, 0, sizeof(uint16_t));
			memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

			if ((fru_offset + MAX_PAYLOAD_LEN) < MAX_EEPROM_SZ)
				read_length = MAX_PAYLOAD_LEN;
			else
			{
				read_length = (MAX_EEPROM_SZ - fru_offset);
			}

			if (read_length > 0)
				if (response = (*i2c_write_read)(handle, slave_addr, (uint8_t)sizeof(uint16_t), write_buffer, read_length, &buffer[buf_idx]) != SUCCESS)
				{
					log_fnc_err(UNKNOWN_ERROR, "read_raw_from_eeprom() i2c_write_read failed.");
					break;
				}

				fru_offset += read_length;
				buf_idx += read_length;
		}
	}
	log_out("\n");

	close_i2c_channel(handle);

	free(buffer);

	print_msg("eeprom read", &response);

	return response;
}

static int check_fru_area_checksum(uint8_t *fru_area_data, uint16_t fru_area_length)
{
	uint8_t checksum = 0;
	size_t i;
	uint8_t fru_area_checksum = fru_area_data[fru_area_length-1];
	for (i=0;i<fru_area_length-1;i++) {
		checksum = (checksum + fru_area_data[i]) % 256;
	}
	checksum = 0x0 - checksum;
	if (fru_area_checksum == checksum)
		return SUCCESS;
	else {
		printf("check_fru_area_checksum error:%d , %d\n", fru_area_checksum, checksum);
		return FAILURE;
	}
}

static int parse_and_check_fru_area_info(FRU_INFO_TYPE em_fru_type)
{
	uint8_t *fru_area_data = NULL;
	uint16_t fru_area_length = 0;
	FRU_AREA_INFO_FIELD *ptr_fru_field = NULL;
	uint16_t checksum = 0;
	int fru_area_idx = 0;
	size_t i;

	if (em_fru_type >= EM_FRU_INFO_MAX) {
		log_fnc_err(UNKNOWN_ERROR, "parse_and_check_fru_area_info error :em_fru_type:%d", em_fru_type);
		return FAILURE;
	}

	fru_area_data = FRU_AREA_HEADER[em_fru_type].area_data;
	fru_area_length = FRU_AREA_HEADER[em_fru_type].area_length;
	ptr_fru_field = FRU_AREA_HEADER[em_fru_type].ptr_fru_field;
	if (fru_area_data == NULL || fru_area_length == 0 || ptr_fru_field == NULL) {
		log_fnc_err(UNKNOWN_ERROR, "parse_and_check_fru_area_info error parameters:em_fru_type:%d", em_fru_type);
		return FAILURE;
	}

	i = 0;
	fru_area_idx = 0;
	while (ptr_fru_field[i].fru_field_name != NULL) {
		switch (ptr_fru_field[i].length) {
			case FRU_FIELD_LENGTH_REFER_PREVIOUS:
				if ((i-1) >= 0 && (ptr_fru_field[i-1].data != NULL)) {
					ptr_fru_field[i].length = *(ptr_fru_field[i-1].data);
					//Type:Bit7~Bit6
					//Length:Bit5~Bit0
					ptr_fru_field[i].length &= ~FRU_AREA_INFO_TYPE_MASK;
				} else {
					 log_fnc_err(UNKNOWN_ERROR, "parse_and_check_fru_area_info error parameters:[%s][%d]",
					 	ptr_fru_field[i].fru_field_name, i);
					return FAILURE;
				}
				break;
			case FRU_FIELD_LENGTH_REMAIN_SPACE:
				ptr_fru_field[i].length = fru_area_length - fru_area_idx - 1; //last byte of fru area must be checksum
				break;
			default:
				break;
		}
		ptr_fru_field[i].data = &fru_area_data[fru_area_idx];
		fru_area_idx+=ptr_fru_field[i].length;
		if (fru_area_idx > fru_area_length) {
			log_fnc_err(UNKNOWN_ERROR, "parse_and_check_fru_area_info fru_area_idx(%d) > fru_area_length(%d)",
					 	fru_area_idx, fru_area_length);
			return FAILURE;
		}
		i+=1;
	}
	return check_fru_area_checksum(fru_area_data, fru_area_length);
}

static void free_fru_area(FRU_INFO_TYPE em_fru_type)
{
	if (FRU_AREA_HEADER[em_fru_type].area_data != NULL) {
		free(FRU_AREA_HEADER[em_fru_type].area_data);
		FRU_AREA_HEADER[em_fru_type].area_data = NULL;
	}
}

static int read_fru_area_info_from_eeprom(int32_t handle, uint8_t slave_addr, FRU_INFO_TYPE em_fru_type)
{
	uint16_t absolute_area_header_offset = 0;
	uint16_t absolute_area_length_offset = 0;
	uint8_t  area_length;

	if (em_fru_type >= EM_FRU_INFO_MAX) {
		log_fnc_err(UNKNOWN_ERROR, "read_fru_area_info_from_eeprom error :em_fru_type:%d \n", em_fru_type);
		return FAILURE;
	}
	//it has no need to read fru area with header offset is zero or fru_field is undefined
	if (FRU_AREA_HEADER[em_fru_type].header_offset == 0 || FRU_AREA_HEADER[em_fru_type].ptr_fru_field == NULL)
		return UNKNOWN_ERROR;

	absolute_area_header_offset = (FRU_AREA_HEADER[em_fru_type].header_offset * FRU_HEADER_OFFSET_MULTIPLY);
	absolute_area_length_offset =  absolute_area_header_offset + FRU_AREA_HEADER[em_fru_type].notify_area_length_offset;
	if ((*i2c_write_read)(handle, slave_addr, sizeof(uint16_t),(uint8_t *) &absolute_area_length_offset, sizeof(uint8_t), &area_length) != SUCCESS) {
		log_fnc_err(UNKNOWN_ERROR, "fru area %d read area length fail.", em_fru_type);
		return FAILURE;
	}
	if (area_length <= 0)
		return UNKNOWN_ERROR;
	FRU_AREA_HEADER[em_fru_type].area_length = area_length * FRU_AREA_HEADER[em_fru_type].area_length_multiply;
	free_fru_area(em_fru_type);
	FRU_AREA_HEADER[em_fru_type].area_data = calloc(FRU_AREA_HEADER[em_fru_type].area_length, sizeof(uint8_t));

	uint8_t  fru_area[area_length];
	memset(fru_area, 0, area_length);
	if ((*i2c_write_read)(handle, slave_addr, sizeof(uint16_t), (uint8_t *) &absolute_area_header_offset,
		 FRU_AREA_HEADER[em_fru_type].area_length, FRU_AREA_HEADER[em_fru_type].area_data) != SUCCESS) {
		log_fnc_err(UNKNOWN_ERROR, "fru area %d read area info fail", em_fru_type);
		free_fru_area(em_fru_type);
		return FAILURE;
	}
	return parse_and_check_fru_area_info(em_fru_type);
}

static void show_fru_area_info(void)
{
	size_t i, j;
	for (i = 0; i < sizeof(FRU_AREA_HEADER)/sizeof(FRU_AREA_INFO_HEADER); i++)
	{
		if (FRU_AREA_HEADER[i].area_data != NULL) {
			switch(FRU_AREA_HEADER[i].em_fru_type) {
				case EM_CHASSIS:
					printf("==== Chassis Info =====\n");
					break;
				case EM_BOARD:
					printf("==== Board Info =====\n");
					break;
				case EM_PRODUCT:
					printf("==== Product Info =====\n");
					break;
				case EM_INTERNAL_USE:
					printf("==== Internal Use Area =====\n");
					break;
				default:
					continue;
					break;
			}
			j = 0;
			FRU_AREA_INFO_FIELD *ptr_fru_field = FRU_AREA_HEADER[i].ptr_fru_field;
			while (ptr_fru_field[j].fru_field_name != NULL) {
				printf("%s:", ptr_fru_field[j].fru_field_name);
				switch (ptr_fru_field[j].em_show_msg_type) {
					case EM_MSG_TYPE_HEX:
					{
						size_t k;
						for (k = 0; k < ptr_fru_field[j].length; k++)
							printf("0x%x ", ptr_fru_field[j].data[k]);
						break;
					}
					case EM_MSG_TYPE_STRING:
					{
						size_t k;
						for (k = 0; k < ptr_fru_field[j].length; k++)
							printf("%c", ptr_fru_field[j].data[k]);
						break;
					}
					case EM_MSG_TYPE_DATE_STRING:
					{
						size_t k;
						for (k = 0; k < ptr_fru_field[j].length; k++)
							printf("0x%x ", ptr_fru_field[j].data[k]);
						break;
					}
					default:
						break;
				}
				printf("\n");
				j+=1;
			}
		}
	}
}

/* reads raw fru data from eeprom */
static int read_from_eeprom(uint8_t channel, uint8_t slave_addr) {

	print_msg("reading from eeprom", NULL);

	uint8_t *buffer = NULL;
	FRU_HEADER header = {0};
	uint16_t fru_offset = 0;
	int response = SUCCESS;
	size_t i;
	int32_t handle = 0;

	buffer = calloc(sizeof(FRU_HEADER), sizeof(uint8_t));
	// open i2c bus
	if ((response = open_i2c_channel(channel, &handle)) != SUCCESS) {
		log_fnc_err(UNKNOWN_ERROR, "unable to open i2c bus");
		return FAILURE;
	}

	// i2c read fru header
	if ((*i2c_write_read)(handle, slave_addr, sizeof(uint16_t), (uint8_t *)&fru_offset, sizeof(FRU_HEADER), buffer) == SUCCESS)
	{
		memcpy(&header, buffer, sizeof(FRU_HEADER));

		FRU_AREA_HEADER[EM_CHASSIS].header_offset = header.chassis;
		FRU_AREA_HEADER[EM_BOARD].header_offset = header.board;
		FRU_AREA_HEADER[EM_PRODUCT].header_offset = header.product;
		FRU_AREA_HEADER[EM_INTERNAL_USE].header_offset = header.areaoffset;

		for (i = 0; i < sizeof(FRU_AREA_HEADER)/sizeof(FRU_AREA_INFO_HEADER); i++)
		{
			if (read_fru_area_info_from_eeprom(handle, slave_addr, FRU_AREA_HEADER[i].em_fru_type) != SUCCESS)
				free_fru_area(FRU_AREA_HEADER[i].em_fru_type);
		}
		show_fru_area_info();
	}
	else {
		log_fnc_err(UNKNOWN_ERROR, "i2c access fru header fail.");
		response = FAILURE;
	}
	close_i2c_channel(handle);
	free(buffer);
	for (i = 0; i < sizeof(FRU_AREA_HEADER)/sizeof(FRU_AREA_INFO_HEADER); i++)
		free_fru_area(FRU_AREA_HEADER[i].em_fru_type);
	return response;
}

/* writes a buffer to eeprom */
static int write_to_eeprom(uint8_t channel, uint8_t slave_addr, uint16_t fru_offset, uint16_t write_length, uint8_t* buffer)
{
	int32_t response = SUCCESS;
	uint8_t chunksize = 0;
	uint8_t write_buf[sizeof(uint16_t)];
	uint16_t write_idx = 0;
	int32_t handle = 0;

	// open i2c bus
	if (open_i2c_channel(channel, &handle) != SUCCESS) {
		log_fnc_err(UNKNOWN_ERROR, "unable to open i2c bus");
		return FAILURE;
	}

	while (write_idx < write_length)
	{
		if ((write_idx + MAX_PAYLOAD_LEN) < write_length)
			chunksize = MAX_PAYLOAD_LEN;
		else
			chunksize = write_length - write_idx;

		// copy the start offset to the write buffer
		memset(write_buf, 0, sizeof(uint16_t));
		memcpy(write_buf, &fru_offset, sizeof(uint16_t));

		if (chunksize > 0)
			if (response = (*i2c_write)(handle, slave_addr, sizeof(uint16_t), write_buf, chunksize, &buffer[write_idx]) != SUCCESS) {
				goto end;
			}

		write_idx += chunksize;
		fru_offset += chunksize;
	}

	end:

	close_i2c_channel(handle);
	return response;
}


/* opens input file and coordinates the write to eeprom */
static int read_file_write_eeprom(uint8_t channel, uint8_t slave_addr, uint8_t* filename)
{
	int rc = SUCCESS;
	uint8_t *fru_data = NULL;
	size_t fru_data_length = 0;
	FRU_HEADER header = {0};
	size_t i;
	if (filename != NULL) {
		FILE *input_file;
		char *mode = "rb";

		input_file = fopen(filename, mode);

		if (input_file == NULL) {
			log_fnc_err(UNKNOWN_ERROR, "can't open input file: %s", filename);
			return FAILURE;
		}

		//get file size for fru_data_length
		fseek(input_file, 0, SEEK_END); //move file end position
		fru_data_length = ftell(input_file); //get file size
		fseek(input_file, 0, SEEK_SET); //back file start position

		fru_data = calloc(fru_data_length, sizeof(uint8_t));
		fread(fru_data, fru_data_length, 1, input_file);
		fclose(input_file);

		//get fru header from buffer
		memcpy(&header, fru_data, sizeof(FRU_HEADER));
		FRU_AREA_HEADER[EM_CHASSIS].header_offset = header.chassis;
		FRU_AREA_HEADER[EM_BOARD].header_offset = header.board;
		FRU_AREA_HEADER[EM_PRODUCT].header_offset = header.product;
		FRU_AREA_HEADER[EM_INTERNAL_USE].header_offset = header.areaoffset;
		for (i = 0; i < sizeof(FRU_AREA_HEADER)/sizeof(FRU_AREA_INFO_HEADER); i++) {
			if (read_fru_from_buffer(fru_data, fru_data_length, FRU_AREA_HEADER[i].em_fru_type) == FAILURE) {
				printf("file is no correct: %d\n", FRU_AREA_HEADER[i].em_fru_type);
				free(fru_data);
				return FAILURE;
			}
		}
		write_to_eeprom(channel, slave_addr, 0, fru_data_length, fru_data);
		free(fru_data);
	}
	else {
		log_fnc_err(UNKNOWN_ERROR, "file not found.");
		rc = UNKNOWN_ERROR;
	}

	return rc;
}

int main(int argc, char **argv)
{
	if (argc <= 3){
		usage();
		return 1;
	}

#ifdef DEBUG
	print_msg("warning debug mode", NULL);
	/* assign fn ptr to debug func */
	i2c_write_read = &i2c_write_read_dbg;
	i2c_write = &i2c_write_dbg;
	debug_buffer = calloc(MAX_EEPROM_SZ, sizeof(uint8_t));
#else
	/* assign fn ptr to prod func */
	i2c_write_read = &i2c_write_read_prod;
	i2c_write = &i2c_write_prod;
#endif // DEBUG

	int response = 0;
	uint8_t channel = 0;
	uint8_t slave_addr = 0;
	uint8_t operation = 0;
	uint8_t *filename = NULL;
	uint8_t raw_read = 0;

	int i;
	for (i = 0; i < argc; i++){

			if (strcmp(argv[i], "-c") == SUCCESS)
				channel = strtol(argv[i + 1], NULL, 16);

			if (strcmp(argv[i], "-s") == SUCCESS)
				slave_addr = strtol(argv[i + 1], NULL, 16);

			if (strcmp(argv[i], "-r") == SUCCESS) {
				operation = 0;
				if (argc > (i + 1)) {
					if (strcmp(argv[i + 1], "raw") == SUCCESS)
						raw_read = 1;
				}
			}

			if (strcmp(argv[i], "-w") == SUCCESS){
				operation = 1;

				if (argc >= (i + 1)) {
					filename = argv[i + 1];
				}
				else {
					usage();
					response = UNKNOWN_ERROR;
					goto main_end;
				}
			}
		}

		if (validate_fru_address != SUCCESS)
		{

			log_out("i2c target: %d %x\n", channel, slave_addr);

			if (operation == 0) {

				if (raw_read == 0) {
					/* read from the target eeprom */
					response = read_from_eeprom(channel, slave_addr);
				}
				else
				{
					response = read_raw_from_eeprom(channel, slave_addr);
				}
			}
			else{
				if (filename != NULL) {
					/* read input file and write it to the eeprom */
					response = read_file_write_eeprom(channel, slave_addr, filename);
#ifdef DEBUG
					/* in debug mode do read back*/
					response = read_from_eeprom(channel, slave_addr);
#endif // DEBUG
				}
				else {
					log_fnc_err(UNKNOWN_ERROR, "File not found.");
					response = UNKNOWN_ERROR;
				}
			}
		}
		else
		{
			log_fnc_err(UNKNOWN_ERROR, "invalid channel or slave address: channel: %x address %x", channel, slave_addr);
			response = UNKNOWN_ERROR;
		}

	main_end:

#ifdef DEBUG
		free(debug_buffer);
		print_msg("warning debug mode - end", NULL);
#endif // DEBUG

		return response;
}
