/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <synaptics_dsx.h>
#include "synaptics_dsx_i2c.h"

#define SYSFS_FOLDER_NAME "rmidb"

static ssize_t rmidb_sysfs_query_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidb_sysfs_control_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidb_sysfs_data_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidb_sysfs_command_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct synaptics_rmi4_db_handle {
	unsigned char intr_mask;
	unsigned char intr_reg_num;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	struct synaptics_rmi4_data *rmi4_data;
	struct kobject *sysfs_dir;
	struct synaptics_rmi4_access_ptr *fn_ptr;
};

static struct device_attribute attrs[] = {
	__ATTR(query_base_addr, S_IRUGO,
			rmidb_sysfs_query_base_addr_show,
			synaptics_rmi4_store_error),
	__ATTR(control_base_addr, S_IRUGO,
			rmidb_sysfs_control_base_addr_show,
			synaptics_rmi4_store_error),
	__ATTR(data_base_addr, S_IRUGO,
			rmidb_sysfs_data_base_addr_show,
			synaptics_rmi4_store_error),
	__ATTR(command_base_addr, S_IRUGO,
			rmidb_sysfs_command_base_addr_show,
			synaptics_rmi4_store_error),
};

static struct synaptics_rmi4_db_handle *rmidb;

DECLARE_COMPLETION(rmidb_remove_complete);

static ssize_t rmidb_sysfs_query_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", rmidb->query_base_addr);
}

static ssize_t rmidb_sysfs_control_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", rmidb->control_base_addr);
}

static ssize_t rmidb_sysfs_data_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", rmidb->data_base_addr);
}

static ssize_t rmidb_sysfs_command_base_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", rmidb->command_base_addr);
}

static int rmidb_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_offset;
	unsigned char page;
	unsigned short addr;
	bool fdb_found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_data *rmi4_data = rmidb->rmi4_data;
	unsigned char temp;


	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
			addr |= (page << 8);

			retval = rmidb->fn_ptr->read(rmi4_data,
					addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			addr &= ~(MASK_8BIT << 8);

			if (!rmi_fd.fn_number)
				break;

			if (rmi_fd.fn_number == SYNAPTICS_RMI4_F51) {
				fdb_found = true;
				goto fdb_found;
			}

			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
		}
	}

	if (!fdb_found) {

		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to find F51\n",
				__func__);
		return -EINVAL;
	}


fdb_found:

	dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: F51 found\n",__func__);
       printk("[TSP-xw]%s:F51 found\n",__func__);
	rmidb->query_base_addr = rmi_fd.query_base_addr | (page << 8);
	rmidb->control_base_addr = rmi_fd.ctrl_base_addr | (page << 8);
	rmidb->data_base_addr = rmi_fd.data_base_addr | (page << 8);
	rmidb->command_base_addr = rmi_fd.cmd_base_addr | (page << 8);
	printk("[TSP-xw]the value of addr is query = 0x%2x,control = 0x%2x,data = 0x%2x,command = 0x%2x.\n",
		rmidb->query_base_addr,rmidb->control_base_addr,rmidb->data_base_addr,rmidb->command_base_addr);
	rmidb->intr_reg_num = (intr_count + 7) / 8;
	if (rmidb->intr_reg_num != 0)
		rmidb->intr_reg_num -= 1;

	rmidb->intr_mask = 0;
	intr_offset = intr_count % 8;
	for (ii = intr_offset;
			ii < ((rmi_fd.intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++) {
		rmidb->intr_mask |= 1 << ii;
	}

	rmi4_data->intr_mask[0] |= rmidb->intr_mask;

	addr = rmi4_data->f01_ctrl_base_addr + 1;

	retval = rmidb->fn_ptr->write(rmi4_data,
			addr,
			&(rmi4_data->intr_mask[0]),
			sizeof(rmi4_data->intr_mask[0]));
	if (retval < 0)
		return retval;

	return 0;
}

extern int synaptics_rmi4_f51_report(struct synaptics_rmi4_data *rmi4_data);
static void synaptics_rmi4_db_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (!rmidb)
		return;
       int retval;
	dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: F51 interrupt triggered\n",__func__);

	retval = synaptics_rmi4_f51_report(rmi4_data);
	if (retval <0)
		printk("[TSP-xw]read f51_0x0400 fail\n");
	/* Do whatever you want here */


	return;
}

static int synaptics_rmi4_db_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;

	rmidb = kzalloc(sizeof(*rmidb), GFP_KERNEL);
	if (!rmidb) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for rmidb\n",
				__func__);

		retval = -ENOMEM;
		goto exit;
	}

	rmidb->fn_ptr = kzalloc(sizeof(*(rmidb->fn_ptr)), GFP_KERNEL);
	if (!rmidb->fn_ptr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto exit_scan_pdt;
	}

	rmidb->rmi4_data = rmi4_data;
	rmidb->fn_ptr->read = rmi4_data->i2c_read;
	rmidb->fn_ptr->write = rmi4_data->i2c_write;
	rmidb->fn_ptr->enable = rmi4_data->irq_enable;

	retval = rmidb_scan_pdt();
	if (retval < 0) {
		retval = 0;
		goto exit_scan_pdt;
	}

	rmidb->sysfs_dir = kobject_create_and_add(SYSFS_FOLDER_NAME,
			&rmi4_data->input_dev->dev.kobj);
	if (!rmidb->sysfs_dir) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs directory\n",
				__func__);
		retval = -ENODEV;
		goto exit_sysfs_dir;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(rmidb->sysfs_dir,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_sysfs_attrs;
		}
	}

	return 0;

exit_sysfs_attrs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(rmidb->sysfs_dir, &attrs[attr_count].attr);

	kobject_put(rmidb->sysfs_dir);

exit_sysfs_dir:
exit_scan_pdt:
	kfree(rmidb);
	rmidb = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_db_remove(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;

	if (!rmidb)
		goto exit;

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(rmidb->sysfs_dir, &attrs[attr_count].attr);

	kobject_put(rmidb->sysfs_dir);

	kfree(rmidb);
	rmidb = NULL;

exit:
	complete(&rmidb_remove_complete);

	return;
}


static void synaptics_rmi4_db_reset(struct synaptics_rmi4_data *rmi4_data)
{
	rmidb_scan_pdt();

	return;
}


static struct synaptics_rmi4_exp_fn rmidb_module = {
	.fn_type = RMI_DEBUG,
	.init = synaptics_rmi4_db_init,
	.remove = synaptics_rmi4_db_remove,
	.reset = synaptics_rmi4_db_reset,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_db_attn,
};

static int __init rmidb_module_init(void)
{
	synaptics_rmi4_new_function(&rmidb_module, true);

	return 0;
}

static void __exit rmidb_module_exit(void)
{
	synaptics_rmi4_new_function(&rmidb_module, false);

	wait_for_completion(&rmidb_remove_complete);

	return;
}

module_init(rmidb_module_init);
module_exit(rmidb_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Debug Module");
MODULE_LICENSE("GPL v2");
