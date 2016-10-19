/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Written by: daniel_hk (https://github.com/danielhk)
 ** 2016/10/12: initial release
 ** 2016/10/18: add CONFIG_MTK_I2C_EXTENSION support for DMA transfer
 */

#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include "AudDrv_tfa9897.h"

#define TFA_I2C_CHANNEL		(1)

#define ECODEC_SLAVE_ADDR_WRITE	0x68
#define ECODEC_SLAVE_ADDR_READ	0x69

/* from cust_gpio_usage.h generated with mt6752's codegen.dws
#define GPIO_SMARTPA_LDO_EN_PIN			(GPIO118 | 0x80000000)
#define GPIO_SMARTPA_LDO_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_LDO_EN_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_LDO_EN_PIN_M_DPI_D		GPIO_MODE_01
#define GPIO_SMARTPA_LDO_EN_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_EINT_PIN			(GPIO123 | 0x80000000)
#define GPIO_SMARTPA_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_EINT_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_EINT_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_RST_PIN			(GPIO125 | 0x80000000)
#define GPIO_SMARTPA_RST_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_RST_PIN_M_CLK		GPIO_MODE_03
#define GPIO_SMARTPA_RST_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_RST_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_I2S_BCK_PIN		(GPIO127 | 0x80000000)
#define GPIO_SMARTPA_I2S_BCK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_BCK_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_I2S_BCK_PIN_M_I2S3_BCK	GPIO_MODE_02
#define GPIO_SMARTPA_I2S_BCK_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_I2S_WS_PIN			(GPIO128 | 0x80000000)
#define GPIO_SMARTPA_I2S_WS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_WS_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_I2S_WS_PIN_M_I2S3_LRCK	GPIO_MODE_02
#define GPIO_SMARTPA_I2S_WS_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_I2S_DOUT_PIN		(GPIO129 | 0x80000000)
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_EINT	GPIO_MODE_04
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_DPI_VSYNC	GPIO_MODE_01
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_I2S3_DO	GPIO_MODE_02
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_DBG_SDA	GPIO_MODE_03
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_DBG_MON_B	GPIO_MODE_07

#define GPIO_SMARTPA_I2S_DIN_PIN		(GPIO133 | 0x80000000)
#define GPIO_SMARTPA_I2S_DIN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_DIN_PIN_M_CLK		GPIO_MODE_03
#define GPIO_SMARTPA_I2S_DIN_PIN_M_EINT		GPIO_MODE_04
#define GPIO_SMARTPA_I2S_DIN_PIN_M_I2S0_DI	GPIO_MODE_01
#define GPIO_SMARTPA_I2S_DIN_PIN_M_AGPS_SYNC	GPIO_MODE_02
#define GPIO_SMARTPA_I2S_DIN_PIN_M_DBG_MON_B	GPIO_MODE_07
*/

#define I2C_MASTER_CLOCK	400
#define TFA9897_I2C_DEVNAME	"tfa9897"

#define AudDrv_tfa9897_NAME	"MediaTek TFA9897 SmartPA Driver"
#define AUDDRV_AUTHOR		"daniel_hk"

// i2c variable
static struct i2c_client *tfa_client = NULL;
#ifdef CONFIG_MTK_I2C_EXTENSION
#define DMA_BUFFER_LENGTH	(4096)
static u8 *Tfa9897DMABuf_va = NULL;		// DMA virtual addr
static dma_addr_t Tfa9897DMABuf_pa = 0;		// DMA physical addr
#else
#define RW_BUFFER_LENGTH	(256)
static char WriteBuffer[RW_BUFFER_LENGTH];
static char ReadBuffer[RW_BUFFER_LENGTH];
#endif

// i2c_device_id & i2c_board_info
static const struct i2c_device_id Tfa9897_i2c_id[] = {
    {TFA9897_I2C_DEVNAME, 0},
    {}
};

static struct i2c_board_info __initdata Tfa9897_dev = {
    .type = TFA9897_I2C_DEVNAME,
    .addr = (ECODEC_SLAVE_ADDR_WRITE >> 1),
};

static struct i2c_board_info Tfa9897_dev_info = {
    .type = TFA9897_I2C_DEVNAME,
    .addr = (ECODEC_SLAVE_ADDR_WRITE >> 1),
};

// function declration
static int Tfa9897Pa_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int Tfa9897Pa_i2c_remove(struct i2c_client *client);
void AudDrv_tfa9897_Init(void);
static int Tfa9897Pa_register(void);
ssize_t NXPSpk_read_byte(u8 addr, u8 *returnData);

// i2c_driver - detect callback
static int Tfa9897Pa_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TFA9897_I2C_DEVNAME);
    return 0;
}

// i2c driver
struct i2c_driver Tfa9897Pa_i2c_driver =
{
    .probe = Tfa9897Pa_i2c_probe,
    .remove = Tfa9897Pa_i2c_remove,
    .detect = Tfa9897Pa_i2c_detect,
    .id_table = Tfa9897_i2c_id,
    .driver = {
        .name = TFA9897_I2C_DEVNAME,
    },
};

// i2c_driver - probe callback
static int Tfa9897Pa_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    tfa_client = client;
    pr_debug("%s: enter, device '%s' , addr %x\n", __func__, id->name, tfa_client->addr);

    // reset the chip
    mt_set_gpio_mode(GPIO_SMARTPA_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_SMARTPA_RST_PIN, GPIO_OUT_ZERO);
    msleep(2);
    mt_set_gpio_out(GPIO_SMARTPA_RST_PIN, GPIO_OUT_ONE);
    msleep(2);
    mt_set_gpio_out(GPIO_SMARTPA_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
#ifdef CONFIG_MTK_I2C_EXTENSION
    // allocate dma buffer
    Tfa9897DMABuf_va = (u8 *)dma_alloc_coherent(&(client->dev),
			DMA_BUFFER_LENGTH, (dma_addr_t *)&Tfa9897DMABuf_pa, GFP_KERNEL);

    if(!Tfa9897DMABuf_va)
    {
	NXP_ERROR("tfa9897 dma_alloc_coherent error\n");
	pr_debug("tfa9897_i2c_probe failed\n");
        return -1;
    }
#endif
    pr_debug("%s success\n", __func__);
    return 0;
}

// i2c_driver - remove callback
static int Tfa9897Pa_i2c_remove(struct i2c_client *client)
{
    tfa_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&Tfa9897Pa_i2c_driver);
#ifdef CONFIG_MTK_I2C_EXTENSION
    if(Tfa9897DMABuf_va)
    {
	dma_free_coherent(NULL, DMA_BUFFER_LENGTH, Tfa9897DMABuf_va, Tfa9897DMABuf_pa);
	Tfa9897DMABuf_va = NULL;
	Tfa9897DMABuf_pa = 0;
    }
#endif
    // @disable the LDO
    mt_set_gpio_mode(GPIO_SMARTPA_LDO_EN_PIN, GPIO_SMARTPA_LDO_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SMARTPA_LDO_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SMARTPA_LDO_EN_PIN, GPIO_OUT_ZERO);
    msleep(1);
    return 0;
}

// Register the platform driver and i2c driver
static int Tfa9897Pa_register()
{
    struct i2c_adapter *adapter;
    struct i2c_client *client;
    int ret = 0;

    pr_debug("%s \n", __func__);

    // @enable the LDO
    mt_set_gpio_mode(GPIO_SMARTPA_LDO_EN_PIN, GPIO_SMARTPA_LDO_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_SMARTPA_LDO_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SMARTPA_LDO_EN_PIN, GPIO_OUT_ONE);
    msleep(1);

    adapter = i2c_get_adapter(TFA_I2C_CHANNEL);
    if (!adapter) {
        NXP_ERROR("%s: i2c_get_adapter(%d) failed\n",__func__,TFA_I2C_CHANNEL);
	// adapter empty, register the board info
        ret = i2c_register_board_info(TFA_I2C_CHANNEL, &Tfa9897_dev, 1);
    } else {
	// allocate a new i2c device
        client = i2c_new_device(adapter, &Tfa9897_dev_info);
        if (!client) {
            NXP_ERROR("%s: i2c_new_device() failed\n", __func__);
            i2c_put_adapter(adapter);
        }
    }
    // add the i2c driver
    if (i2c_add_driver(&Tfa9897Pa_i2c_driver))
    {
        pr_debug("fail to add driver to i2c");
        return -1;
    }
    return ret;
}

// Init the chip
void AudDrv_tfa9897_Init(void)
{
    pr_debug("Set GPIO according to stock values \n");
    mt_set_gpio_mode(GPIO_SMARTPA_I2S_WS_PIN, GPIO_SMARTPA_I2S_WS_PIN_M_I2S3_LRCK);
    mt_set_gpio_mode(GPIO_SMARTPA_I2S_BCK_PIN, GPIO_SMARTPA_I2S_BCK_PIN_M_I2S3_BCK);
    mt_set_gpio_mode(GPIO_SMARTPA_I2S_DOUT_PIN, GPIO_SMARTPA_I2S_DOUT_PIN_M_I2S3_DO);
    mt_set_gpio_mode(GPIO_SMARTPA_I2S_DIN_PIN, GPIO_SMARTPA_I2S_DIN_PIN_M_I2S0_DI);
}

// file_operations - unlocked_ioctl callback
static long AudDrv_tfa9897_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    struct i2c_client *client = fp->private_data;

    pr_debug("%s: cmd = 0x%x arg = %lu\n",__func__, cmd, arg);

    switch (cmd)
    {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
	   /* NOTE:  devices set up to work with "new style" drivers
	    * can't use I2C_SLAVE, even when the device node is not
	    * bound to a driver.  Only I2C_SLAVE_FORCE will work.
	    *
	    * Setting the PEC flag here won't affect kernel drivers,
	    * which will be using the i2c_client node registered with
	    * the driver model core.  Likewise, when that client has
	    * the PEC flag already set, the i2c-dev driver won't see
	    * (or use) this setting.
	    */
	    if ((arg > 0x3ff) ||
		(((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
		    return -EINVAL;

	    if (cmd == I2C_SLAVE && client->addr==arg)
		return -EBUSY;
	    /* REVISIT: address could become busy later */
	    return 0;
        default:
        {
	   /* NOTE:  returning a fault code here could cause trouble
	    * in buggy userspace code.  Some old kernel bugs returned
	    * zero in this case, and userspace code might accidentally
	    * have depended on that bug.
	    */
            pr_debug("%s: Illegal command: %x \n",__func__, cmd);
            ret = -ENOTTY;
            break;
        }
    }
    return ret;
}

// platform_driver - probe callback
static int AudDrv_tfa9897_probe(struct platform_device *dev)
{
    pr_debug("%s \n", __func__);

    // register the i2c driver
    Tfa9897Pa_register();
    // Init the chip's GPIO
    AudDrv_tfa9897_Init();
#ifndef CONFIG_MTK_I2C_EXTENSION
    // clear the R/W buffers
    memset((void *)WriteBuffer, 0, RW_BUFFER_LENGTH);
    memset((void *)ReadBuffer, 0, RW_BUFFER_LENGTH);
#endif
    pr_debug("-%s \n", __func__);
    return 0;
}

static int AudDrv_tfa9897_open(struct inode *inode, struct file *fp)
{
    return 0;
}

#ifdef CONFIG_MTK_I2C_EXTENSION
// send data to the chip
static int nxp_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
    int ret;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg;

    pr_debug("+%s: count=%d\n", __func__, count);
    msg.timing = I2C_MASTER_CLOCK;
    msg.flags = client->flags & I2C_M_TEN;
    msg.len = count;
    msg.buf = (char *)buf;
    msg.addr = client->addr & I2C_MASK_FLAG;
    msg.ext_flag = client->ext_flag;
    if(count > 8) {
	msg.addr |= I2C_ENEXT_FLAG;
	msg.ext_flag |= I2C_DMA_FLAG;
    }
    ret = i2c_transfer(adap, &msg, 1);

    // ret == 1 means 1 msg transmitted, set ret = count, otherwise ret keep the error code
    pr_debug("-%s: ret=%d\n", __func__, ret);
    return (ret == 1) ? count : ret;
}

// receive data from the chip
static int nxp_i2c_master_recv(const struct i2c_client *client, char *buf, int count)
{
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg;
    int ret;

    pr_debug("+%s count=%d\n", __func__, count);
    msg.timing = I2C_MASTER_CLOCK;
    msg.flags = client->flags & I2C_M_TEN;
    msg.flags |= I2C_M_RD;
    msg.len = count;
    msg.buf = (char *)buf;
    msg.addr = client->addr & I2C_MASK_FLAG;
    msg.ext_flag = client->ext_flag;
    if(count > 8) {
	msg.addr |= I2C_ENEXT_FLAG;
	msg.ext_flag |= I2C_DMA_FLAG;
    }
    ret = i2c_transfer(adap, &msg, 1);

    // ret == 1 means 1 msg transmitted, set ret = count, otherwise ret keep the error code
    pr_debug("-%s: ret=%d\n", __func__, ret);
    return (ret == 1) ? count : ret;
}
#endif

// file_operations - Write callback
static ssize_t AudDrv_tfa9897_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)
{
#ifdef CONFIG_MTK_I2C_EXTENSION
    char *tmp;
#endif
    int ret;

    pr_debug("+%s, count=%d\n", __func__, (int)count);
#ifdef CONFIG_MTK_I2C_EXTENSION
    tmp = kmalloc(count,GFP_KERNEL);
    if (tmp==NULL)
	return -ENOMEM;
    if (copy_from_user(tmp,data,count)) {
	kfree(tmp);
	return -EFAULT;
    }

    memcpy(Tfa9897DMABuf_va, tmp, count);

    if (count <= 8)
	ret = nxp_i2c_master_send(tfa_client, tmp, count);
    else
	ret = nxp_i2c_master_send(tfa_client, (char *)Tfa9897DMABuf_pa, count);

    kfree(tmp);
#else
    if (copy_from_user(WriteBuffer,data,count))
	return -EFAULT;

    if (count <= 8)
	ret = i2c_master_send(tfa_client, WriteBuffer, count);
    else
	ret = i2c_master_send(tfa_client, (unsigned char *)(uintptr_t) WriteBuffer, count);
#endif
    pr_debug("-%s: ret=%d\n", __func__, ret);
    return ret;
}

// file_operations - read callback
static ssize_t AudDrv_tfa9897_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
#ifdef CONFIG_MTK_I2C_EXTENSION
    char *tmp;
#endif
    int ret;

    pr_debug("+%s, count=%d\n", __func__, (int)count);
#ifdef CONFIG_MTK_I2C_EXTENSION
    if (count > DMA_BUFFER_LENGTH)
	count = DMA_BUFFER_LENGTH;

    tmp = kmalloc(count,GFP_KERNEL);
    if (tmp==NULL)
	return -ENOMEM;

    if(count <= 8)
	ret = nxp_i2c_master_recv(tfa_client, tmp, count);
    else {
	ret = nxp_i2c_master_recv(tfa_client,(char *)Tfa9897DMABuf_pa,count);
	memcpy(tmp, Tfa9897DMABuf_va, count);
    }
    if (ret >= 0)
	ret = copy_to_user(data,tmp,count)?-EFAULT:ret;
    kfree(tmp);
#else
    if (count <= 8)
	ret = i2c_master_recv(tfa_client, ReadBuffer, count);
    else
	ret = i2c_master_recv(tfa_client, (unsigned char *)(uintptr_t) ReadBuffer, count);

    if (ret >= 0)
	ret = copy_to_user(data, ReadBuffer, count) ? -EFAULT : ret;
#endif
    pr_debug("-%s: ret=%d\n", __func__, ret);
    return ret;
}

/**************************************************************************
 *  The misc device and its file operations
 **************************************************************************/

static struct file_operations AudDrv_tfa9897_fops = {
    .owner = THIS_MODULE,
    .open = AudDrv_tfa9897_open,
    .unlocked_ioctl = AudDrv_tfa9897_ioctl,
    .write = AudDrv_tfa9897_write,
    .read = AudDrv_tfa9897_read,
};

static struct miscdevice AudDrv_tfa9897_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = TFA9897_I2C_DEVNAME,
    .fops = &AudDrv_tfa9897_fops,
};

/***************************************************************************
 *  AudDrv_tfa9897_mod_init / AudDrv_tfa9897_mod_exit
 *  Module init and exit (only called at system boot up)
 **************************************************************************/

#ifdef CONFIG_OF
static const struct of_device_id mtk_tfa9897_of_ids[] = {
    { .compatible = "mediatek,tfa9897", },
    {}
};
#endif

static struct platform_driver AudDrv_tfa9897 = {
    .probe = AudDrv_tfa9897_probe,
    .driver = {
        .name = TFA9897_I2C_DEVNAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = mtk_tfa9897_of_ids,
#endif
    },
};

static int AudDrv_tfa9897_mod_init(void)
{
    int ret = 0;
    pr_debug("+%s\n", __func__);

    // Register platform DRIVER
    ret = platform_driver_register(&AudDrv_tfa9897);
    if (ret)
    {
        pr_debug("%s: platform_driver_register Fail: %d \n", __func__, ret);
        return ret;
    }

    // register MISC device
    if ((ret = misc_register(&AudDrv_tfa9897_device)))
    {
        pr_debug("%s: misc_register Fail:%d \n", __func__, ret);
        return ret;
    }

    pr_debug("-%s\n", __func__);
    return 0;
}
module_init(AudDrv_tfa9897_mod_init);

static void AudDrv_tfa9897_mod_exit(void)
{
    pr_debug("+%s \n", __func__);

    pr_debug("-%s \n", __func__);
}
module_exit(AudDrv_tfa9897_mod_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AudDrv_tfa9897_NAME);
MODULE_AUTHOR(AUDDRV_AUTHOR);
