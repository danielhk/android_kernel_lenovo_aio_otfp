/*
 * Driver for EEPROM
 *
 *
 */

#if 0
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "eeprom.h"
#include "eeprom_define.h"
#include "GT24c32a.h"
#include <asm/system.h>  // for SMP
#else

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "GT24c08.h"

#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
//#include <asm/system.h>  // for SMP
#endif
//#define EEPROMGETDLT_DEBUG
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB printk
#else
#define EEPROMDB(x,...)
#endif


static DEFINE_SPINLOCK(g_GT24c08_EEPROMLock); // for SMP
#define EEPROM_I2C_BUSNUM 3
static struct i2c_board_info __initdata kd_eeprom_dev={ I2C_BOARD_INFO("GT24C08_CAM_CAL_DRV", 0x50)};

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "GT24C08_CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/


/*******************************************************************************
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 3
#define Boundary_Address 256
#define EEPROM_Address_Offset 0xC


#define SHADINGSIZE 644
#define SHADINGFLAG (0xc5)

#define SHADING_START 0x41
#define SHADING_PAGE0_SIZE 0xBF
#define SHADING_PAGE1_SIZE 0x100
#define SHADING_PAGE2_SIZE 0xC5

/*******************************************************************************
*
********************************************************************************/
static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pEEPROM_CharDrv = NULL;
//static spinlock_t g_EEPROMLock;
//spin_lock(&g_EEPROMLock);
//spin_unlock(&g_EEPROMLock);

static struct class *EEPROM_class = NULL;
static atomic_t g_EEPROMatomic;


/*******************************************************************************
*
********************************************************************************/
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iBurstReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int kdSetI2CBusNum(u32 i2cBusNum) ;

/*******************************************************************************
*
********************************************************************************/
// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt6516.c which is 8 bytes
int iWriteEEPROM(u16 a_u2Addr  , u32 a_u4Bytes, u8 * puDataInBytes)
{
    int  i4RetValue = 0;
    u32 u4Index = 0;
    char puSendCmd[8] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,
        0, 0, 0, 0, 0, 0};
    if(a_u4Bytes + 2 > 8)
    {
        EEPROMDB("[GT24c08] exceed I2c-mt65xx.c 8 bytes limitation (include address 2 Byte)\n");
        return -1;
    }

    for(u4Index = 0 ; u4Index < a_u4Bytes ; u4Index += 1 )
    {
        puSendCmd[(u4Index + 2)] = puDataInBytes[u4Index];
    }
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));
    if (i4RetValue != (a_u4Bytes + 2))
    {
        EEPROMDB("[GT24c08] I2C write  failed!! \n");
        return -1;
    }
    mdelay(10); //for tWR singnal --> write data form buffer to memory.
    
   //EEPROMDB("[EEPROM] iWriteEEPROM done!! \n");
    return 0;
}


// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes
int iReadEEPROM(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
	#if 1
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    //EEPROMDB("[EEPROM] iReadEEPROM!! \n");

    if(ui4_length > 8)
    {
        EEPROMDB("[GT24c08] exceed I2c-mt65xx.c 8 bytes limitation\n");
        return -1;
    }
    //spin_lock(&g_GT24c08_EEPROMLock); //for SMP
   // g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);	
    //spin_unlock(&g_GT24c08_EEPROMLock); // for SMP
    
    EEPROMDB("[EEPROM] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
    if (i4RetValue != 2)
    {
        EEPROMDB("[EEPROM] I2C send read address failed!! \n");
        return -1;
    }

    EEPROMDB("[EEPROM] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        EEPROMDB("[EEPROM] I2C read data failed!! \n");
        return -1;
    }
    //spin_lock(&g_GT24c08_EEPROMLock); //for SMP
    //g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;    
   // spin_unlock(&g_GT24c08_EEPROMLock); // for SMP    
#else
	int  i4RetValue = 0;
	kdSetI2CBusNum(3);
	i4RetValue=iReadReg(a_u2Addr,a_puBuff,GT24C08_DEVICE_ID);

	if (i4RetValue !=0)
	{
		EEPROMDB("[CAM_CAL] I2C read data failed!! \n");
		return -1;
	}	
	EEPROMDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);

#endif
    EEPROMDB("[EEPROM] iReadEEPROM done!! \n");
    return 0;
}


int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;

   EEPROMDB("[GT24c08] iWriteData\n" );

   EEPROMDB("[GT24c08] iWriteData done\n" );
 
   return 0;
}



int iReadDataFromGT24c08(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{

		char puSendCmd[2];// = {(char)(ui4_offset & 0xFF) };
		unsigned short SampleOffset = (unsigned short)((ui4_offset) & (0x0000FFFF)) ;
        unsigned short EEPROM_Address[3] = {0xA0,0xA2,0xA4} ;
        //unsigned char address_offset = ((SampleNum *SampleOffset) + EEPROM_Address_Offset+ ui4_length) / Boundary_Address;
        short loop[2], loopCount;
        unsigned short SampleCount;
        u8 * pBuff ;
        u32 u4IncOffset = 0;
        int  i4RetValue = 0;


        pBuff = pinputdata;
        
        EEPROMDB("[EEPROM] ui4_offset=%x ui4_offset(80)=%x ui4_offset(8)=%x\n",ui4_offset , (unsigned short)( (ui4_offset>>8) & 0x0000FFFF),SampleOffset);
        
        
        EEPROMDB("[EEPROM] EEPROM_Address[0]=%x EEPROM_Address[1]=%x,EEPROM_Address[2]=%x\n",EEPROM_Address[0] ,EEPROM_Address[1],EEPROM_Address[2]);

        //loop[0] = (Boundary_Address * address_offset) - ((SampleNum *SampleOffset) + EEPROM_Address_Offset);
        loop[0] = ((ui4_length>>4)<<4);

        loop[1] = ui4_length - loop[0] ;

        
        EEPROMDB("[EEPROM] loop[0]=%d loop[1]=%d\n",(loop[0]) ,(loop[1]));

        puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF) ;
        puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF) ;

        for(loopCount=0; loopCount < Read_NUMofEEPROM; loopCount++)
        {
               do 
               {
                   if( 16 <= loop[loopCount])
                   {
                       
                       EEPROMDB("[EEPROM]1 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x, EEPROM(%x)\n",loopCount ,loop[loopCount],puSendCmd[0],puSendCmd[1],EEPROM_Address[loopCount] );
                       iReadRegI2C(puSendCmd , 2, (u8*)pBuff,16,EEPROM_Address[loopCount]);
                      // i4RetValue=iBurstReadRegI2C(puSendCmd , 2, (u8*)pBuff,16,EEPROM_Address[loopCount]);
   
                       u4IncOffset += 16;
                       loop[loopCount] -= 16;
                       //puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ;
                       puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF) ;
                       puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF) ;
                       pBuff = pinputdata + u4IncOffset;
                   }
                   else if(0 < loop[loopCount])
                   {
                       EEPROMDB("[EEPROM]2 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x \n",loopCount ,loop[loopCount],puSendCmd[0],puSendCmd[1] );
                       iReadRegI2C(puSendCmd , 2, (u8*)pBuff,loop[loopCount],EEPROM_Address[loopCount]);
                       //i4RetValue=iBurstReadRegI2C(puSendCmd , 2, (u8*)pBuff,16,EEPROM_Address[loopCount]);

                       u4IncOffset += loop[loopCount];
                       loop[loopCount] -= loop[loopCount];
                       //puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ;
                       puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF) ;
                       puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF) ;
                       pBuff = pinputdata + u4IncOffset;
                   }
               }while (loop[loopCount] > 0);
        }

   return 0;
}

int iReadEEPROM_CAL_WR(u8 a_u2Addr, u32 ui4_length, u8 * a_puBuff,u8 pageNum)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {a_u2Addr};
    g_pstI2Cclient->addr = (GT24C08_DEVICE_ID + pageNum)>>1;
    g_pstI2Cclient->ext_flag=((g_pstI2Cclient->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG ;//write and read mode
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, (1<<8)|ui4_length);
    g_pstI2Cclient->ext_flag=((g_pstI2Cclient->ext_flag ) & I2C_MASK_FLAG )  ;//write and read mode
    if (i4RetValue < 0)
    {
        EEPROMDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }
    else
    {
      memcpy(a_puBuff,puReadCmd,ui4_length);
    }
    return 0;
}

kal_uint8 EEPROMReadSensor(u8 address,u8 pageNum)
{
	u8 getByte;
	s8 ret;
	//ret = iReadEEPROM_CAL(address, 1, &getByte,pageNum);
	ret = iReadEEPROM_CAL_WR(address, 1, &getByte,pageNum);
	EEPROMDB("address=0x%x, getByte= 0x%x\n", address, getByte);
	return getByte;
}




int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   #if 0
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;
   if (ui4_offset + ui4_length >= 0x2000)
   {
      EEPROMDB("[GT24c08] Read Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
   do 
   {
       if(i4ResidueDataLength >= 8)
       {
           i4RetValue = iReadEEPROM((u16)u4CurrentOffset, 8, pBuff);
           if (i4RetValue != 0)
           {
                EEPROMDB("[EEPROM] I2C iReadData failed!! \n");
                return -1;
           }           
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           i4RetValue = iReadEEPROM((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                EEPROMDB("[EEPROM] I2C iReadData failed!! \n");
                return -1;
           }  
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
//   EEPROMDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
//   EEPROMDB("[S24EEPORM] iReadData done\n" );
	#else
	int  i4RetValue = 0;
	u8 u4Offset = 0;
	
	for(u4Offset = 0;u4Offset<ui4_length;u4Offset++)
	{
		*(pinputdata+u4Offset) = EEPROMReadSensor(ui4_offset+u4Offset, 0);
	}
	#endif
   return 0;
}

int iReadShadingEEPROMData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	int  i4RetValue = 0;
	int  i4ResidueDataLength;
	u8 u4afOffset = 0;
	int ret=0;
	u16 u4shadingoffset = 0,page_size=0;
	u8 rdRdyStatus, arrayLength;
	u8 GroupBank[504]={0};
 	u8 shadingflag=0;
	EEPROMDB("[CAM_CAL] iReadShadingEEPROMData start!! \n");
	//page0
	for (u4shadingoffset = SHADING_START; u4shadingoffset < (SHADING_PAGE0_SIZE+SHADING_START); u4shadingoffset++)
	{
		*(pinputdata+u4shadingoffset-SHADING_START) = EEPROMReadSensor(u4shadingoffset, 0);
	}
	//page 1
	for (u4shadingoffset = 0 ; u4shadingoffset < SHADING_PAGE1_SIZE; u4shadingoffset++)
	{
		*(pinputdata+u4shadingoffset+SHADING_PAGE0_SIZE) = EEPROMReadSensor(u4shadingoffset, 2);
	}
	
	for (u4shadingoffset = 0 ; u4shadingoffset < SHADING_PAGE2_SIZE; u4shadingoffset++)
	{
		*(pinputdata+u4shadingoffset+SHADING_PAGE0_SIZE+SHADING_PAGE1_SIZE) = EEPROMReadSensor(u4shadingoffset, 4);
	}
	EEPROMDB("[CAM_CAL] iReadShadingEEPROMData end!! \n");
	//module id
	
	return 0;
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long EEPROM_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
    ssize_t writeSize;
    u8 readTryagain=0;

#ifdef EEPROMGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            EEPROMDB("[GT24c08] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[GT24c08] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        EEPROMDB("[GT24c08] ioctl allocate mem failed\n");
        return -ENOMEM;
    }

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        EEPROMDB("[GT24c08] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            EEPROMDB("[GT24c08] Write CMD \n");
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            EEPROMDB("[GT24c08] Read CMD \n");
#ifdef EEPROMGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif 
            EEPROMDB("[EEPROM] offset %x \n", ptempbuf->u4Offset);
            EEPROMDB("[EEPROM] length %x \n", ptempbuf->u4Length);
        EEPROMDB("[EEPROM] Before read Working buffer address 0x%p\n", pWorkingBuff);
	
	if(ptempbuf->u4Offset==0x23)
	{
		i4RetValue = iReadData((u16)(ptempbuf->u4Offset), ptempbuf->u4Length, pWorkingBuff);
	}
	else if(ptempbuf->u4Offset==0x41)
	{
		i4RetValue = iReadShadingEEPROMData(0,0,pWorkingBuff);
	}
            EEPROMDB("[GT24c08] After read Working buffer data  0x%4x \n", *pWorkingBuff);


            break;
        default :
      	     EEPROMDB("[GT24c08] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        EEPROMDB("[GT24c08] to user length %d \n", ptempbuf->u4Length);
        EEPROMDB("[GT24c08] to user  Working buffer address 0x%p \n", pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            EEPROMDB("[GT24c08] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int EEPROM_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    EEPROMDB("[GT24c08] EEPROM_Open\n");
    spin_lock(&g_GT24c08_EEPROMLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_GT24c08_EEPROMLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_EEPROMatomic,0);
    }
    spin_unlock(&g_GT24c08_EEPROMLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int EEPROM_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_GT24c08_EEPROMLock);

    g_u4Opened = 0;

    atomic_set(&g_EEPROMatomic,0);

    spin_unlock(&g_GT24c08_EEPROMLock);

    return 0;
}
#ifdef CONFIG_COMPAT
static int GT24c08_compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int GT24c08_compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long GT24c08_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    EEPROMDB("[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	  EEPROMDB("[CAMERA SENSOR] GT24c08_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = GT24c08_compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = GT24c08_compat_put_cal_info_struct(data32, data);


        if(err != 0)
            EEPROMDB("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif

static const struct file_operations g_stEEPROM_fops =
{
    .owner = THIS_MODULE,
    .open = EEPROM_Open,
    .release = EEPROM_Release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = GT24c08_Ioctl_Compat,
#endif
    .unlocked_ioctl = EEPROM_Ioctl
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterEEPROMCharDrv(void)
{
    struct device* EEPROM_device = NULL;

#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_EEPROMdevno, 0, 1,EEPROM_DRVNAME) )
    {
        EEPROMDB("[GT24c08] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_EEPROMdevno , 1 , EEPROM_DRVNAME) )
    {
        EEPROMDB("[GT24c08] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pEEPROM_CharDrv = cdev_alloc();

    if(NULL == g_pEEPROM_CharDrv)
    {
        unregister_chrdev_region(g_EEPROMdevno, 1);

        EEPROMDB("[GT24c08] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

    g_pEEPROM_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1))
    {
        EEPROMDB("[GT24c08] Attatch file operation failed\n");

        unregister_chrdev_region(g_EEPROMdevno, 1);

        return -EAGAIN;
    }

    EEPROM_class = class_create(THIS_MODULE, "GT24C08_EEPROM");
    if (IS_ERR(EEPROM_class)) {
        int ret = PTR_ERR(EEPROM_class);
        EEPROMDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);

    return 0;
}

inline static void UnregisterEEPROMCharDrv(void)
{
    //Release char driver
    cdev_del(g_pEEPROM_CharDrv);

    unregister_chrdev_region(g_EEPROMdevno, 1);

    device_destroy(EEPROM_class, g_EEPROMdevno);
    class_destroy(EEPROM_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int EEPROM_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int EEPROM_i2c_remove(struct i2c_client *);

static const struct i2c_device_id EEPROM_i2c_id[] = {{EEPROM_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {EEPROM_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver EEPROM_i2c_driver = {
    .probe = EEPROM_i2c_probe,                                   
    .remove = EEPROM_i2c_remove,                           
//   .detect = EEPROM_i2c_detect,                           
    .driver.name = EEPROM_DRVNAME,
    .id_table = EEPROM_i2c_id,                             
};

#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, EEPROM_DRVNAME);
    return 0;
}
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    EEPROMDB("[GT24c08] Attach I2C \n");
//    spin_lock_init(&g_EEPROMLock);

    //get sensor i2c client
    spin_lock(&g_GT24c08_EEPROMLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = GT24C08_DEVICE_ID>>1;
    spin_unlock(&g_GT24c08_EEPROMLock); // for SMP    
    
    EEPROMDB("[EEPROM] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterEEPROMCharDrv();

    if(i4RetValue){
        EEPROMDB("[GT24c08] register char device failed!\n");
        return i4RetValue;
    }


    EEPROMDB("[GT24c08] Attached!! \n");
    return 0;                                                                                       
} 

static int EEPROM_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int EEPROM_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&EEPROM_i2c_driver);
}

static int EEPROM_remove(struct platform_device *pdev)
{
    i2c_del_driver(&EEPROM_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stEEPROM_Driver = {
    .probe		= EEPROM_probe,
    .remove	= EEPROM_remove,
    .driver		= {
        .name	= EEPROM_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stEEPROM_Device = {
    .name = EEPROM_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init EEPROM_i2C_init(void)
{
    i2c_register_board_info(EEPROM_I2C_BUSNUM, &kd_eeprom_dev, 1);
    if(platform_driver_register(&g_stEEPROM_Driver)){
        EEPROMDB("failed to register GT24c08 driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stEEPROM_Device))
    {
        EEPROMDB("failed to register GT24c08 driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit EEPROM_i2C_exit(void)
{
	platform_driver_unregister(&g_stEEPROM_Driver);
}

module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);

MODULE_DESCRIPTION("EEPROM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


