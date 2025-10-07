#ifndef _SERIAL_SCREEN_H
#define _SERIAL_SCREEN_H

#include "head_file.h"

#define SERIAL_SCREEN_RX_LEN	6
#define SERIAL_SCREEN_TX_LEN	19


typedef __packed struct
{
	uint8_t header;
	uint8_t type;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t ender;
}Serial_Screen_Rx_Data_t;


typedef __packed struct
{
	uint8_t hander[8];		// 0x74 0x30 0x2e 0x74 0x78 0x74 0x3d 0x22
	uint8_t	data1[3];		
	uint8_t sum_buffer;		// 0x2b
	uint8_t	data2[3];
	uint8_t ender[4];		// 0x22 0xff 0xff 0xff
}Serial_Screen_Tx_Data_t;

typedef struct
{
	Serial_Screen_Rx_Data_t Serial_Screen_Rx_Data;
	Serial_Screen_Tx_Data_t Serial_Screen_Tx_Data;
	uint8_t rx_data[SERIAL_SCREEN_RX_LEN];
	uint8_t tx_data[SERIAL_SCREEN_TX_LEN];
}serial_screen_t;

void serial_screen_send(uint8_t data[6]);
void serial_screen_init(void);
void Serial_Screen_Communication_Callback(void);

void get_car_id(uint8_t *car_id);

#endif
