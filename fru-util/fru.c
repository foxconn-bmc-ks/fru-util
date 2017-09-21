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
static int read_from_eeprom(uint8_t channel, uint8_t slave_addr, uint8_t *result);
static int write_to_eeprom(uint8_t channel, uint8_t slave_addr, uint16_t fru_offset, uint16_t write_length, uint8_t* buffer);

/* debug buffer */
uint8_t *debug_buffer;


/* function pointer for i2c write read */
int(*i2c_write_read)(int32_t handle,
	uint8_t slave_addr,
	uint8_t write_length,
	uint8_t* write_data,
	uint8_t read_length,
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
static const uint8_t CHASSIS_FRU_FIELDS[CHASSIS_INFO_COUNT][MAX_NAME_LEN] = {
	"Chassis_Info_Format_Version:",
	"Chassis_Info_Area_Length:",
	"Chassis_Type:",
	"Chassis_PartNum_Type_Length:",
	"Chassis_PartNum:",
	"Chassis_SerialNum_Type_Length:",
	"Chassis_SerialNum:",
	"Chassis_Custom_Field_1_Type_Length:",
	"Chassis_Custom_Field_1:",
	"Chassis_Custom_Field_2_Type_Length:",
	"Chassis_Custom_Field_2:",
	"Chassis_No_More:",
	"Chassis_PAD:",
	"Chassis_CheckSum:"
};

/* First prop represent offset */
/* Second prop 0: string format, 1: hex */
static const uint8_t CHASSIS_FIELDS[CHASSIS_INFO_COUNT][CHASSIS_INFO_PROP] = {
	{0x0, 1},
	{0x1, 1},
	{0x2, 1},
	{0x3, 1},
	{0x4, 0},
	{0x24, 1},
	{0x25, 0},
	{0x45, 1},
	{0x46, 0},
	{0x4E, 1},
	{0x4F, 0},
	{0x52, 1},
	{0x53, 1},
	{0x57, 1}
};

static const uint8_t BOARD_FRU_FIELDS[BOARD_INFO_COUNT][MAX_NAME_LEN] = {
	"Board_Area_Format_Version:",
	"Board_Area_Length:",
	"Board_Language_Code:",
	"Board_Mfg_Time:",
	"Board_Manufacturer_Type_Length:",
	"Board_Manufacturer:",
	"Board_Product_Name_Type_Length:",
	"Board_Product_Name:",
	"Board_Serial_Number_Type_Length:",
	"Board_Serial_Number:",
	"Board_Part_Number_Type_Length:",
	"Board_Part_Number:",
	"Board_FRU_File_ID_Type_Length:",
	"Board_FRU_File_ID:",
	"Board_Custom_Field_1_Type_Length:",
	"Board_Custom_Field_1:",
	"Board_Indicate_No_More_Info:",
	"Board_PAD:",
	"Board_Area_Checksum:"
};
static const uint8_t BOARD_FIELDS[BOARD_INFO_COUNT][CHASSIS_INFO_PROP] = {
	{0x0, 1},
	{0x1, 1},
	{0x2, 1},
	{0x3, 1},
	{0x6, 1},
	{0x7, 0},
	{0x17, 1},
	{0x18, 0},
	{0x20, 1},
	{0x21, 0},
	{0x41, 1},
	{0x42, 0},
	{0x62, 1},
	{0x63, 0},
	{0x6D, 1},
	{0x6E, 0},
	{0x74, 1},
	{0x75, 1},
	{0x77, 1}
};
static const uint8_t PRODUCT_FRU_FIELDS[PRODUCT_INFO_COUNT][MAX_NAME_LEN] = {
	"Product_Area_Format_Version:",
	"Product_Area_Length:",
	"Product_Language_Code:",
	"Product_Manufacturer_Type_Length:",
	"Product_Manufacturer:",
	"Product_Name_Type_Length:",
	"Product_Name:",
	"Product_Part_Model_Number_Type_Length:",
	"Product_Part_Model_Number:",
	"Product_Version_Type_Length:",
	"Product_Version:",
	"Product_Serial_Number_Type_Length:",
	"Product_Serial_Number:",
	"Product_Asset_Tag_Type_Length:",
	"Product_Asset_Tag:",
	"Product_FRU_File_ID_Type_Length:",
	"Product_FRU_File_ID:",
	"Product_Reserverd:",
	"Product_Custom_Field_1_Type_Length:",
	"Product_Custom_Field_1:",
	"Product_Custom_Field_2_Type_Length:",
	"Product_Custom_Field_2:",
	"Product_Indicate_No_More_Info:",
	"Product_PAD:",
	"Product_Area_Checksum:"
};
static const uint8_t PRODUCT_FIELDS[PRODUCT_INFO_COUNT][CHASSIS_INFO_PROP] = {
	{0x0, 1},
	{0x1, 1},
	{0x2, 1},
	{0x3, 1},
	{0x4, 0},
	{0x14, 1},
	{0x15, 0},
	{0x1D, 1},
	{0x1E, 0},
	{0x3E, 1},
	{0x3F, 0},
	{0x47, 1},
	{0x48, 0},
	{0x68, 1},
	{0x69, 0},
	{0x6C, 1},
	{0x6D, 1},
	{0x6E, 1},
	{0x6F, 1},
	{0x70, 0},
	{0x73, 1},
	{0x74, 0},
	{0x77, 1},
	{0x78, 1},
	{0x7F, 1}
};
/* converts input char buffer to binary data */
static int convert_binary_data(uint8_t* buffer) {

	if (buffer == NULL)
		return FAILURE;

	uint64_t hex = strtoul(buffer, NULL, 16);
	memset(buffer, 0, sizeof(long));

	memcpy(buffer, &hex, sizeof(long));

	return SUCCESS;
}

/*
requited fru fields.  tags must appear in the fru file.
*/
static AREA_FIELD fru_field(uint16_t *idx, uint8_t *buffer)
{
	AREA_FIELD field;

	field.length = &buffer[++(*idx)];
	field.data = &buffer[++(*idx)];

	*idx += *field.length;

	return field;
}

static int collect_fru_area_info(uint8_t *buffer,
		uint16_t address, const uint8_t fru_fields[][MAX_NAME_LEN],
		const uint8_t fields[][CHASSIS_INFO_PROP],
		uint8_t info_count, int16_t area_size) {

	uint8_t *value = calloc(area_size, sizeof(uint8_t));
	memcpy(value, &buffer[address+fields[0][0]], area_size);
	uint8_t field_length = 0;
	uint8_t checksum = 0, count = 0;
	uint8_t i, j;
	char hex_string[256];

	for (i=0;i<=fields[info_count-1][0];i++) {
		checksum = (checksum + buffer[address+i]) % 256;
	}
	if (checksum == 0) {
		for(i=0;i<info_count;i++) {
			if (i+1 < info_count)
				field_length = fields[i+1][0] - fields[i][0];
			else
				field_length = area_size - fields[i][0];

			memset(value, '\0', area_size);
			memcpy(value, &buffer[address+fields[i][0]], field_length);
			if (fields[i][1] == 0)
				log_out("%s%s", fru_fields[i], value);
			else {
				count = 0;
				for (j=0;j<field_length;j++) {
					count += snprintf(hex_string+count, sizeof(hex_string)/sizeof(hex_string[0])-count,  "%.2X", *value);
				}
				log_out("%s%s", fru_fields[i], hex_string);
			}
			/* If area length is 0 then stop to parse this FRU area
				The second field for each info area is represent area length */
			if (i == 1 && buffer[address+1] == 0)
				break;
			memset(value, '\0', area_size);
		}
	} else {
		log_out("%sERROR", fru_fields[info_count-1]);
		free(value);
		return FAILURE;
	}
	free(value);
	return SUCCESS;
}
/*
	assigns fru charactor data in buffer to fru structures.
*/
static int read_fru_from_buffer(uint8_t *buffer, uint16_t length)
{
	uint16_t idx;
	uint8_t field_length = 0;
	char hex_string[256];

	collect_fru_area_info(buffer, EEPROM_CHASSIS_ADDRESS,
			CHASSIS_FRU_FIELDS, CHASSIS_FIELDS, CHASSIS_INFO_COUNT,
			CHASSIS_AREA_SIZE);
	collect_fru_area_info(buffer, EEPROM_BOARD_ADDRESS,
			BOARD_FRU_FIELDS, BOARD_FIELDS, BOARD_INFO_COUNT,
			BOARD_AREA_SIZE);
	collect_fru_area_info(buffer, EEPROM_PRODUCT_ADDRESS,
			PRODUCT_FRU_FIELDS, PRODUCT_FIELDS, PRODUCT_INFO_COUNT,
			PRODUCT_AREA_SIZE);

	return SUCCESS;
}

unsigned char* hexstr_to_char(const char* hexstr)
{
	size_t len = strlen(hexstr), i, j;
	size_t final_len = len / 2;
	if (len % 2 != 0)
		return 0;
	unsigned char* chrs = (unsigned char*)malloc((final_len+1) * sizeof(*chrs));
	for (i=0, j=0; j<final_len; i+=2, j++)
		chrs[j] = (hexstr[i] % 32 + 9) % 25 * 16 + (hexstr[i+1] % 32 + 9) % 25;
	chrs[final_len] = '\0';
	return chrs;
}

static int set_checksum(uint8_t *fru_data, uint16_t address,
		const uint8_t fields[][CHASSIS_INFO_PROP],
		uint8_t info_count) {
	uint8_t checksum = 0;
	size_t i;
	for (i=0;i<fields[info_count-1][0];i++) {
		checksum = (checksum + fru_data[address+i]) % 256;
	}
	checksum = 0x0 - checksum;
	strncpy(&fru_data[address + fields[info_count-1][0]], &checksum, 1);
}

static int find_field_from_file(uint8_t *fru_data, uint8_t *line,
		uint16_t address,
		const uint8_t fru_fields[][MAX_NAME_LEN],
		const uint8_t fields[][CHASSIS_INFO_PROP],
		uint8_t info_count, int16_t area_size, size_t *found) {
	uint8_t tag[MAX_NAME_LEN];
	memset(&tag, 0, MAX_NAME_LEN);
	size_t i;
	uint8_t tag_length = 0, value_length = 0;
	uint16_t base_addr = 0;
	uint8_t field_length = 0;
	int rc = 0;
	for (i = 0; i < info_count && *found==0; i++)
	{
		if (strstr(line, fru_fields[i]) != NULL) {
			*found = 1;
			base_addr = address;

			tag_length = (uint8_t)strlen(fru_fields[i]);

			memset(&tag, 0, MAX_NAME_LEN);
			strncpy(tag, fru_fields[i], tag_length);

			/* check fru file tag format */
			if (strcmp(tag, fru_fields[i]) != 0)
			{
				log_out("header mismatch line beginging: %s\n", line);
				return FAILURE;
			}

			/* replace any new line */
			if (line[strlen(line) - 1] == NEW_LINE) {
				line[strlen(line) - 1] = '\0';
			}
			if (i+1 < info_count)
				field_length = fields[i+1][0] - fields[i][0];
			else
				field_length = area_size - fields[i][0];

			/*
				Verify field have enough length to store given value to FRU
				Value of ordinary string type is stored as-is, so the number of byte it will occupy exactly its string length.
				Value of hex string type will be converted to integer before storing to FRU.
				Since there is no 0x prefix, the number of byte it will occupy is its string length divide by 2.
				Note: here we use the coincidence that value type code plus 1 happens to be the denominator
			*/
			value_length = strlen(line)-tag_length;
			if (field_length >= value_length/(fields[i][1]+1) &&
				value_length % (fields[i][1]+1) == 0) {
				if (fields[i][1] == 1) {
					strncpy(&fru_data[base_addr + fields[i][0]], hexstr_to_char(&line[tag_length]), field_length);
				} else if (fields[i][1] == 0) {
					strncpy(&fru_data[base_addr + fields[i][0]], &line[tag_length], strlen(line));
				}
			} else {
				log_out("field value exceed for FRU designated EEPROM space: %s", line);
				return FAILURE;
			}
			break;
		}
	}
	return SUCCESS;
}

/*
	reads fru text data from file into array
*/
static int read_fru_from_file(uint8_t channel, uint8_t slave_addr, FILE *input, uint8_t operation)
{
	/* fru spec rev 1.3: field lenght: 5:0 */
	/* record maximum lenght 63 bytes */
	uint8_t line[256];
	memset(&line, 0, sizeof(line));

	uint16_t idx = sizeof(FRU_HEADER);

	FRU_HEADER header;
	memset(&header, 0, sizeof(FRU_HEADER));
	header.commonheader = 0x1;
	header.areaoffset = 0;
	header.board = EEPROM_BOARD_ADDRESS / 8;
	header.chassis = EEPROM_CHASSIS_ADDRESS / 8;
	header.product = EEPROM_PRODUCT_ADDRESS / 8;
	header.multirecord = EEPROM_MULTIRECORD_ADDRESS / 8;
	header.pad = 0;
	header.checksum = 0x0 - header.commonheader - header.areaoffset \
				- header.chassis - header.board - header.product - header.multirecord \
				- header.pad;

	uint8_t *fru_data;
	fru_data = calloc(MAX_EEPROM_SZ, sizeof(uint8_t));
	if (operation == OP_PATCH) {
		print_msg("Read From EEPRON", NULL);
		read_from_eeprom(channel, slave_addr, fru_data);
	}
	uint16_t base_addr = 0;

	uint8_t *boardLength;
	uint8_t *prodLength;

	int rc = 0;

	size_t i, found=0;
	while (fgets(line, (MAX_LENGTH + MAX_NAME_LEN), input) != NULL)
	{
		found = 0;
		rc = find_field_from_file(fru_data, line, EEPROM_CHASSIS_ADDRESS,
			CHASSIS_FRU_FIELDS, CHASSIS_FIELDS, CHASSIS_INFO_COUNT,
			CHASSIS_AREA_SIZE, &found);
		if (rc != SUCCESS) {
			free(fru_data);
			return rc;
		}
		rc = find_field_from_file(fru_data, line, EEPROM_BOARD_ADDRESS,
			BOARD_FRU_FIELDS, BOARD_FIELDS, BOARD_INFO_COUNT,
			BOARD_AREA_SIZE, &found);
		if (rc != SUCCESS) {
			free(fru_data);
			return rc;
		}
		rc = find_field_from_file(fru_data, line, EEPROM_PRODUCT_ADDRESS,
			PRODUCT_FRU_FIELDS, PRODUCT_FIELDS, PRODUCT_INFO_COUNT,
			PRODUCT_AREA_SIZE, &found);
		if (rc != SUCCESS) {
			free(fru_data);
			return rc;
		}
	}

	set_checksum(fru_data, EEPROM_CHASSIS_ADDRESS,
		CHASSIS_FIELDS, CHASSIS_INFO_COUNT);
	set_checksum(fru_data, EEPROM_BOARD_ADDRESS,
		BOARD_FIELDS, BOARD_INFO_COUNT);
	set_checksum(fru_data, EEPROM_PRODUCT_ADDRESS,
		PRODUCT_FIELDS, PRODUCT_INFO_COUNT);

	if (rc == SUCCESS)
	{

		/* copy the header */
		if (operation == OP_WRITE)
			memcpy(fru_data, &header, sizeof(FRU_HEADER));

#ifdef DEBUG

		print_msg("read input file buffer", NULL);

		/* read file back */
		rc = read_fru_from_buffer(fru_data, idx);

		print_msg("read buffer", &rc);

#endif // DEBUG

		//* write the eeprom content to eeprom */
		if (rc == SUCCESS)
		{
			uint16_t fru_offset = 0;
			print_msg("write to eeprom", NULL);
			rc = write_to_eeprom(channel, slave_addr, fru_offset, MAX_EEPROM_SZ, fru_data);
			print_msg("write", &rc);
		}
	}
	free(fru_data);
	return rc;
}

/* debug simulating i2c_write_read */
static int i2c_write_read_dbg(int32_t handle, uint8_t slave_addr,
	uint8_t write_length, uint8_t* write_data, uint8_t read_length, uint8_t* buffer)
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
	uint8_t write_length, uint8_t* write_data, uint8_t read_length, uint8_t* buffer)
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

/* supports fru read, by reading fru area from eeprom */
static int read_fru_area(int32_t handle, uint8_t slave_addr, uint16_t *fru_offset,
	uint16_t *buf_idx, int16_t *area_length, uint8_t *buffer)
{
	int response = SUCCESS;
	uint16_t length = 0;
	uint16_t boundary = 0;
	uint16_t fru_idx = 0;
	uint8_t read_length = 0;
	uint8_t write_buffer[sizeof(uint16_t)];

	AREA_HEADER area_header;
	memset(&area_header, 0, sizeof(AREA_HEADER));

	// copy the start offset to the write buffer
	memset(&write_buffer, 0, sizeof(uint16_t));
	memcpy(write_buffer, fru_offset, sizeof(uint16_t));

	/* ensure fru header is withn the page, or read across page boundaries */
	if (MAX_PAGE_SIZE - (*fru_offset % MAX_PAGE_SIZE) > sizeof(AREA_HEADER)) {
		response = (*i2c_write_read)(handle, slave_addr, (uint8_t)sizeof(uint16_t), write_buffer, sizeof(AREA_HEADER), &buffer[*buf_idx]);
	}
	else{
		uint16_t i = 0;
		uint16_t tmp_idx = *buf_idx;
		uint16_t offset = *fru_offset;

		for (; i < sizeof(area_header); i++) {
			if (response = (*i2c_write_read)(handle, slave_addr, (uint8_t)sizeof(uint16_t), write_buffer, sizeof(uint8_t), &buffer[tmp_idx]) != SUCCESS) {
				log_fnc_err(UNKNOWN_ERROR, "area head read error (%d)", response);
				return FAILURE;
			}

			offset++;
			tmp_idx++;

			memcpy(write_buffer, &offset, sizeof(uint16_t));
		}

	}

	if (response == SUCCESS)
	{
		memcpy(&area_header, &buffer[*buf_idx], sizeof(AREA_HEADER));
		*fru_offset += sizeof(AREA_HEADER);
		*buf_idx += sizeof(AREA_HEADER);

		*area_length += (area_header.length * 8);
		if(*area_length > MAX_EEPROM_SZ/2)
		{
			log_fnc_err(UNKNOWN_ERROR, "area length (%d) exceeded %d\n", *area_length, MAX_EEPROM_SZ/2);
			return FAILURE;
		}

		if (area_header.length != 0)
		{
			length = ((area_header.length * 8) - sizeof(AREA_HEADER));

			while (fru_idx < length)
			{
				// copy the start offset to the write buffer
				memset(&write_buffer, 0, sizeof(uint16_t));
				memcpy(write_buffer, fru_offset, sizeof(uint16_t));

				if ((fru_idx + MAX_PAYLOAD_LEN) < length)
					read_length = MAX_PAYLOAD_LEN;
				else
					read_length = (length - fru_idx);

				/* get page boundary */
				boundary = MAX_PAGE_SIZE - (*fru_offset % MAX_PAGE_SIZE);

				if (read_length > boundary)
					read_length = boundary;

				if (read_length > 0)
					if ((*i2c_write_read)(handle, slave_addr, (uint8_t)sizeof(uint16_t), write_buffer, read_length, &buffer[*buf_idx]) != SUCCESS)
					{
						log_fnc_err(UNKNOWN_ERROR, "read_fru_area() i2c_write_read failed.");
						return FAILURE;
					}

				fru_idx += read_length;
				*fru_offset += read_length;
				*buf_idx += read_length;
			}
		}
	}

	return response;
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

	close_i2c_channel(handle);

	free(buffer);

	print_msg("eeprom read", &response);

	return response;
}

/* reads raw fru data from eeprom */
static int read_from_eeprom(uint8_t channel, uint8_t slave_addr, uint8_t *result) {

	print_msg("reading from eeprom", NULL);

	uint8_t *buffer;
	buffer = calloc(MAX_EEPROM_SZ, sizeof(uint8_t));

	FRU_HEADER header;
	memset(&header, 0, sizeof(FRU_HEADER));

	uint16_t fru_offset = 0;
	uint16_t buf_idx = 0;
	uint8_t  write_buffer[sizeof(uint16_t)];

	int response = 0;
	uint16_t length = 0;
	// copy the start offset to the write buffer
	memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

	int32_t handle = 0;
	// open i2c bus
	if ((response = open_i2c_channel(channel, &handle)) != SUCCESS)
		log_fnc_err(UNKNOWN_ERROR, "unable to open i2c bus");

	if (response == SUCCESS) {
		// i2c read fru header
		if ((response = (*i2c_write_read)(handle, slave_addr, sizeof(uint16_t), write_buffer, sizeof(FRU_HEADER), &buffer[buf_idx])) == SUCCESS)
		{
			memcpy(&header, &buffer[buf_idx], sizeof(FRU_HEADER));

			if (header.chassis != 0)
			{
				// update the offset
				fru_offset = (header.chassis * 8);
				buf_idx = (header.chassis * 8);

				// copy the start offset to the write buffer
				memset(&write_buffer, 0, sizeof(uint16_t));
				memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

				if (fru_offset > MAX_EEPROM_SZ)
				{
					log_fnc_err(UNKNOWN_ERROR, "chassis offset (%d) exceeded %d\n", fru_offset, MAX_EEPROM_SZ);
				}
				else
				{
					if (response = read_fru_area(handle, slave_addr, &fru_offset, &buf_idx, &length, buffer) != SUCCESS)
						log_fnc_err(UNKNOWN_ERROR, "fru area chassis read_fru_area() i2c_write_read failed.");
				}
			}
			if (header.board != 0)
			{
				// update the offset
				fru_offset = (header.board * 8);
				buf_idx = (header.board * 8);

				// copy the start offset to the write buffer
				memset(&write_buffer, 0, sizeof(uint16_t));
				memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

				if (fru_offset > MAX_EEPROM_SZ)
				{
					log_fnc_err(UNKNOWN_ERROR, "board (%d) exceeded %d\n", fru_offset, MAX_EEPROM_SZ);
				}
				else
				{
					if (response = read_fru_area(handle, slave_addr, &fru_offset, &buf_idx, &length, buffer) != SUCCESS)
						log_fnc_err(UNKNOWN_ERROR, "fru area chassis read_fru_area() i2c_write_read failed.");
				}
			}

			if (header.product != 0)
			{
				// update the offset
				fru_offset = (header.product * 8);
				buf_idx = (header.product * 8);

				// copy the start offset to the write buffer
				memset(&write_buffer, 0, sizeof(uint16_t));
				memcpy(write_buffer, &fru_offset, sizeof(uint16_t));

				if (fru_offset > MAX_EEPROM_SZ)
				{
					log_fnc_err(UNKNOWN_ERROR, "product (%d) exceeded %d\n", fru_offset, MAX_EEPROM_SZ);
				}
				else
				{
					if (response = read_fru_area(handle, slave_addr, &fru_offset, &buf_idx, &length, buffer) != SUCCESS)
						log_fnc_err(UNKNOWN_ERROR, "fru area chassis read_fru_area() i2c_write_read failed.");
				}
			}

			if (response == SUCCESS) {
				response = read_fru_from_buffer(buffer, length);
            }
		}
	}

	close_i2c_channel(handle);


	if (result != NULL)
		memcpy(result, buffer, MAX_EEPROM_SZ);
	free(buffer);

	return response;
}

/* writes a buffer to eeprom */
static int write_to_eeprom(uint8_t channel, uint8_t slave_addr, uint16_t fru_offset, uint16_t write_length, uint8_t* buffer)
{
	int32_t response = SUCCESS;
	uint8_t chunksize = 0;
	uint8_t write_buf[sizeof(uint16_t)];
	uint16_t write_idx = 0;

	if (write_length > MAX_EEPROM_SZ){
		log_fnc_err(UNKNOWN_ERROR, "fru data length cannot exceed maximum write length: %d.", MAX_EEPROM_SZ);
		return FAILURE;
	}

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
			if (response = (*i2c_write)(handle, slave_addr, sizeof(uint16_t), write_buf, chunksize, &buffer[write_idx]) != SUCCESS)
				goto end;

		write_idx += chunksize;
		fru_offset += chunksize;
	}

	end:

	close_i2c_channel(handle);

	return response;
}

/* opens input file and coordinates the write to eeprom */
static int read_file_write_eeprom(uint8_t channel, uint8_t slave_addr, uint8_t* filename, uint8_t operation)
{
	int rc;
	if (filename != NULL) {
		FILE *input_file;
		char *mode = "r";

		input_file = fopen(filename, mode);

		if (input_file == NULL) {
			log_fnc_err(UNKNOWN_ERROR, "can't open input file: %s", filename);
			return FAILURE;
		}

		rc = read_fru_from_file(channel, slave_addr, input_file, operation);

		if (input_file != NULL)
			fclose(input_file);
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
				operation = OP_READ;
				if (argc > (i + 1)) {
					if (strcmp(argv[i + 1], "raw") == SUCCESS)
						raw_read = 1;
				}
			} else if (strcmp(argv[i], "-w") == SUCCESS){
				operation = OP_WRITE;

				if (argc > (i + 1)) {
					filename = argv[i + 1];
				}
				else {
					usage();
					response = UNKNOWN_ERROR;
					goto main_end;
				}
			} else if (strcmp(argv[i], "-p") == SUCCESS){
				operation = OP_PATCH;

				if (argc > (i + 1)) {
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

			if (operation == OP_READ) {

				if (raw_read == 0) {
					/* read from the target eeprom */
					response = read_from_eeprom(channel, slave_addr, NULL);
				}
				else
				{
					response = read_raw_from_eeprom(channel, slave_addr);
				}
			}
			else{
				if (filename != NULL) {
					/* read input file and write it to the eeprom */
					response = read_file_write_eeprom(channel, slave_addr, filename, operation);
#ifdef DEBUG
					/* in debug mode do read back*/
					response = read_from_eeprom(channel, slave_addr, NULL);
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
