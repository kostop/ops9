#include "robot_com.h"
#include "usart.h"
#include "delay.h"
#include <string.h>
#include "crc8_crc16.h"

#define ROBOT_TX_DATA_LEN			14U
#define ROBOT_RX_DATA_LEN			2U
#define ROBOT_TX_HEADER				0x5CU
#define ROBOT_RX_HEADER				0xC5U
#define ROBOT_BUS_GUARD_CYCLES		1U
#define ROBOT_RESET_TX_WAIT_MS		3U

typedef enum
{
	ROBOT_RX_WAIT_HEADER = 0,
	ROBOT_RX_WAIT_DATA
} robot_rx_state_t;

typedef struct
{
	uint8_t receive_buffer;
	uint8_t receive_state;
	volatile uint8_t tx_busy;
	volatile uint8_t rx_restart_pending;
	volatile uint8_t bus_guard_count;
	volatile uint32_t uart_error_count;
	volatile uint32_t last_uart_error;
	volatile uint32_t tx_deferred_count;
	volatile uint32_t rx_restart_fail_count;
} robot_com_runtime_t;

robot_com_para_t robot_com_para;
uint8_t robot_tx_data[ROBOT_TX_DATA_LEN];
uint8_t robot_rx_data[ROBOT_RX_DATA_LEN];

static robot_com_runtime_t robot_com_runtime;

static void Robot_com_feed_back(void);
static HAL_StatusTypeDef Robot_com_start_receive(void);
static void Robot_com_try_restart_receive(void);

static HAL_StatusTypeDef Robot_com_start_receive(void)
{
	return HAL_UART_Receive_IT(&huart2, &robot_com_runtime.receive_buffer, 1U);
}

static void Robot_com_try_restart_receive(void)
{
	if((robot_com_runtime.rx_restart_pending == 0U) ||
	   (robot_com_runtime.tx_busy != 0U) ||
	   (huart2.RxState != HAL_UART_STATE_READY))
	{
		return;
	}

	__HAL_UART_CLEAR_OREFLAG(&huart2);
	if(Robot_com_start_receive() == HAL_OK)
	{
		robot_com_runtime.rx_restart_pending = 0U;
	}
	else
	{
		robot_com_runtime.rx_restart_fail_count++;
	}
}

void Robot_com_init(void)
{
	memset(&robot_com_runtime, 0, sizeof(robot_com_runtime));
	robot_com_runtime.receive_state = ROBOT_RX_WAIT_HEADER;
	robot_com_runtime.bus_guard_count = ROBOT_BUS_GUARD_CYCLES;
	if(Robot_com_start_receive() != HAL_OK)
	{
		robot_com_runtime.rx_restart_pending = 1U;
		robot_com_runtime.rx_restart_fail_count++;
	}

	robot_com_para.robot_com_control_point = get_control_para_point();
	robot_com_para.robot_com_imu_point = get_imu_para_point();
	robot_com_para.Robot_TX_data.header = ROBOT_TX_HEADER;
}

static void Robot_com_feed_back(void)
{
	robot_com_para.Robot_TX_data.x = robot_com_para.robot_com_control_point->chassis_pos.x;
	robot_com_para.Robot_TX_data.y = robot_com_para.robot_com_control_point->chassis_pos.y;
	robot_com_para.Robot_TX_data.z = robot_com_para.robot_com_control_point->chassis_pos.z;
	memcpy(robot_tx_data, &robot_com_para.Robot_TX_data, ROBOT_TX_DATA_LEN);
	append_CRC8_check_sum(robot_tx_data, ROBOT_TX_DATA_LEN);
}

void Robot_com_send_data(void)
{
	HAL_StatusTypeDef status;

	Robot_com_try_restart_receive();
	if((robot_com_runtime.tx_busy != 0U) || (huart2.gState != HAL_UART_STATE_READY))
	{
		robot_com_runtime.tx_deferred_count++;
		return;
	}

	if(robot_com_runtime.bus_guard_count > 0U)
	{
		robot_com_runtime.bus_guard_count--;
		return;
	}

	Robot_com_feed_back();
	robot_com_runtime.tx_busy = 1U;
	status = HAL_UART_Transmit_IT(&huart2, robot_tx_data, ROBOT_TX_DATA_LEN);
	if(status != HAL_OK)
	{
		robot_com_runtime.tx_busy = 0U;
		robot_com_runtime.tx_deferred_count++;
	}
}

void Robot_com_tx_complete_callback(void)
{
	robot_com_runtime.tx_busy = 0U;
	robot_com_runtime.bus_guard_count = ROBOT_BUS_GUARD_CYCLES;
	Robot_com_try_restart_receive();
}

void Robot_com_error_callback(void)
{
	robot_com_runtime.last_uart_error = huart2.ErrorCode;
	robot_com_runtime.uart_error_count++;
	robot_com_runtime.receive_state = ROBOT_RX_WAIT_HEADER;
	robot_com_runtime.bus_guard_count = ROBOT_BUS_GUARD_CYCLES;
	robot_com_runtime.rx_restart_pending = 1U;

	HAL_UART_AbortReceive(&huart2);
	Robot_com_try_restart_receive();
}

void Robot_com_call_back(void)
{
	robot_com_runtime.bus_guard_count = ROBOT_BUS_GUARD_CYCLES;
	switch(robot_com_runtime.receive_state)
	{
		case ROBOT_RX_WAIT_HEADER:
			if(robot_com_runtime.receive_buffer == ROBOT_RX_HEADER)
			{
				robot_rx_data[0] = ROBOT_RX_HEADER;
				robot_com_runtime.receive_state = ROBOT_RX_WAIT_DATA;
			}
			break;

		case ROBOT_RX_WAIT_DATA:
			if(robot_com_runtime.receive_buffer == ROBOT_RX_HEADER)
			{
				robot_rx_data[0] = ROBOT_RX_HEADER;
			}
			else
			{
				robot_rx_data[1] = robot_com_runtime.receive_buffer;
				memcpy(&robot_com_para.Robot_RX_data, robot_rx_data, ROBOT_RX_DATA_LEN);
				robot_com_runtime.receive_state = ROBOT_RX_WAIT_HEADER;
			}
			break;

		default:
			robot_com_runtime.receive_state = ROBOT_RX_WAIT_HEADER;
			break;
	}

	if(Robot_com_start_receive() != HAL_OK)
	{
		robot_com_runtime.rx_restart_pending = 1U;
		robot_com_runtime.rx_restart_fail_count++;
	}
}

void Robot_com_prepare_reset(void)
{
	uint8_t wait_ms = ROBOT_RESET_TX_WAIT_MS;

	while(((robot_com_runtime.tx_busy != 0U) ||
	       (huart2.gState != HAL_UART_STATE_READY)) && (wait_ms > 0U))
	{
		delay_ms(1U);
		wait_ms--;
	}

	HAL_UART_Abort(&huart2);
	robot_com_runtime.tx_busy = 0U;
	robot_com_runtime.rx_restart_pending = 0U;
}

uint8_t get_ops_cmd_data(void)
{
	return robot_com_para.Robot_RX_data.data;
}

