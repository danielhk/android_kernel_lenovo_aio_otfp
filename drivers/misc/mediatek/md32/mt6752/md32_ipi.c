/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/barrier.h>
#include <mach/mt_reg_base.h>
#include <mach/md32_helper.h>
#include <mach/md32_ipi.h>
#include "md32_irq.h"


struct ipi_desc ipi_desc[MD32_NR_IPI];
struct share_obj *md32_send_obj, *md32_rcv_obj;
struct mutex md32_ipi_mutex;

extern unsigned char *md32_send_buff;
extern unsigned char *md32_recv_buff;

static void ipi_md2host(void)
{
	HOST_TO_MD32_REG = 0x1;
}

void md32_ipi_handler(void)
{
	pr_debug("md32_ipi_handler %d\n", md32_rcv_obj->id);
	if (ipi_desc[md32_rcv_obj->id].handler && md32_rcv_obj->id < MD32_NR_IPI) {
		memcpy_from_md32(md32_recv_buff, (void *)md32_rcv_obj->share_buf,
				 md32_rcv_obj->len);
		ipi_desc[md32_rcv_obj->id].handler(md32_rcv_obj->id, md32_recv_buff,
						   md32_rcv_obj->len);
	} else {
		pr_err("[MD32]md32 ipi ID abnormal or handler is null\n");
		pr_err("[MD32]md32_rcv_obj address = 0x%p\n", md32_rcv_obj);
		pr_err("[MD32]md32_send_obj address = 0x%p\n", md32_send_obj);
	}
	MD32_TO_SPM_REG = 0x0;
	pr_debug("md32_ipi_handler %d done\n", md32_rcv_obj->id);
}

void md32_ipi_init(void)
{
	mutex_init(&md32_ipi_mutex);
	md32_rcv_obj = MD32_DTCM;
	md32_send_obj = md32_rcv_obj + 1;
	pr_debug("md32_rcv_obj = 0x%p\n", md32_rcv_obj);
	pr_debug("md32_send_obj = 0x%p\n", md32_send_obj);
	memset(md32_send_obj, 0, SHARE_BUF_SIZE);
}

/*
  @param id:       IPI ID
  @param handler:  IPI handler
  @param name:     IPI name
*/
ipi_status md32_ipi_registration(ipi_id id, ipi_handler_t handler, const char *name)
{
	if (id < MD32_NR_IPI) {
		ipi_desc[id].name = name;

		if (handler == NULL)
			return ERROR;

		ipi_desc[id].handler = handler;
		return DONE;
	} else {
		return ERROR;
	}
}

/*
  @param id:       IPI ID
  @param buf:      the pointer of data
  @param len:      data length
  @param wait:     If true, wait (atomically) until data have been gotten by Host
*/
ipi_status md32_ipi_send(ipi_id id, void *buf, unsigned int len, unsigned int wait)
{
	unsigned int sw_rstn;

	sw_rstn = readl(MD32_BASE);
	if (sw_rstn == 0x0) {
		pr_debug("md32_ipi_send: MD32 not enabled, ipi id = %d\n", id);
		md32_aee_force_dump();
		return ERROR;
	}

	if (id < MD32_NR_IPI) {
		if (len > sizeof(md32_send_obj->share_buf) || buf == NULL)
			return ERROR;

		if (HOST_TO_MD32_REG)
			return BUSY;

		mutex_lock(&md32_ipi_mutex);


		if (HOST_TO_MD32_REG) {
			mutex_unlock(&md32_ipi_mutex);
			return BUSY;
		}

		memcpy(md32_send_buff, buf, len);

		memcpy_to_md32((void *)md32_send_obj->share_buf, md32_send_buff, len);
		md32_send_obj->len = len;
		md32_send_obj->id = id;
		dsb();
		ipi_md2host();

		if (wait)
			while (HOST_TO_MD32_REG)
				;

		mutex_unlock(&md32_ipi_mutex);
	} else
		return ERROR;

	return DONE;
}
