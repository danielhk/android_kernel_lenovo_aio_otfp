/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for CM36686 als/ps sensor chip.
 */
#ifndef __CM36686_H__
#define __CM36686_H__

#include <linux/ioctl.h>

/*cm36686 als/ps sensor register related macro*/
#define CM36686_REG_ALS_CONF 		0X00
#define CM36686_REG_ALS_THDH 		0X01
#define CM36686_REG_ALS_THDL 		0X02
#define CM36686_REG_PS_CONF1_2		0X03
#define CM36686_REG_PS_CONF3_MS	0X04
#define CM36686_REG_PS_CANC		0X05
#define CM36686_REG_PS_THDL		0X06
#define CM36686_REG_PS_THDH		0X07
#define CM36686_REG_PS_DATA		0X08
#define CM36686_REG_ALS_DATA		0X09
#define CM36686_REG_INT_FLAG		0X0B
#define CM36686_REG_ID_MODE			0X0C

/*CM36686 related driver tag macro*/
#define CM36686_SUCCESS			0
#define CM36686_ERR_I2C			-1
#define CM36686_ERR_STATUS			-3
#define CM36686_ERR_SETUP_FAILURE	-4
#define CM36686_ERR_GETGSENSORDATA	-5
#define CM36686_ERR_IDENTIFICATION	-6

/*----------------------------------------------------------------------------*/
typedef enum{
	CM36686_NOTIFY_PROXIMITY_CHANGE = 1,
}CM36686_NOTIFY_TYPE;
/*----------------------------------------------------------------------------*/
typedef enum{
	CM36686_CUST_ACTION_SET_CUST = 1,
	CM36686_CUST_ACTION_CLR_CALI,
	CM36686_CUST_ACTION_SET_CALI,
	CM36686_CUST_ACTION_SET_PS_THRESHODL
}CM36686_CUST_ACTION;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint16_t    action;
}CM36686_CUST;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint16_t    action;
	uint16_t    part;
	int32_t    data[0];
}CM36686_SET_CUST;
/*----------------------------------------------------------------------------*/
typedef CM36686_CUST CM36686_CLR_CALI;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint16_t    action;
	int32_t     cali;
}CM36686_SET_CALI;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint16_t    action;
	int32_t     threshold[2];
}CM36686_SET_PS_THRESHOLD;
/*----------------------------------------------------------------------------*/
typedef union
{
	uint32_t                    data[10];
	CM36686_CUST                cust;
	CM36686_SET_CUST            setCust;
	CM36686_CLR_CALI            clearCali;
	CM36686_SET_CALI            setCali;
	CM36686_SET_PS_THRESHOLD    setPSThreshold;
}CM36686_CUST_DATA;
/*----------------------------------------------------------------------------*/
#endif
