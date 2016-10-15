#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <generated/autoconf.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <mach/power_loss_test.h>
#include <mach/power_loss_emmc_test.h>
#include <mach/board.h>
#include <mt_sd.h>

extern struct msdc_host *mtk_msdc_host[];

#if defined(PWR_LOSS_SPOH) && defined(CONFIG_MTK_EMMC_SUPPORT)
#define TAG                 "[MVG_TEST]:"
/*
#define MSDC0_MSDC_INT      (MSDC_0_BASE+OFFSET_MSDC_INT)
#define MSDC0_SDC_STS       (MSDC_0_BASE+OFFSET_SDC_STS)
*/
//#define SDC_STS_SDCBUSY     (0x1  << 0)

static struct hrtimer mvg_emmc_hrtimer;
static struct timespec t_hr_start,t_hr_end1;
static unsigned int hrtimer_delay_ns=0;
static unsigned int hrtimer_len=0;
static unsigned int hrtimer_addr=0;
static volatile unsigned int mvg_emmc_reset_planned=0;
static volatile unsigned int mvg_emmc_cancel_reset=0;
static volatile unsigned int mvg_emmc_reset_timeout=0;
static unsigned int mvg_reset_scheduled=0;


extern unsigned int upmu_get_qi_vemc_3v3_en(void); //Refer to platform/mt65xx/drivers/power/upmu_common.c
extern void wdt_arch_reset(char mode);

struct mvg_spoh_emmc_priv_type mvg_spoh_emmc_priv ={
    .reset_time_divisor = 1,
    .emmc_reset_mode = MVG_EMMC_RESET_WO_POWEROFF,
    .emmc_reset_time_mode = MVG_EMMC_RESET_LEN_DEPEND,
    .erase_set = 0
};

static unsigned int w_dealy_max_tbl[MVG_EMMC_PERFTABLE_SIZE];

void mvg_emmc_nanodelay(u32 delay_ns)
{
    //struct timespec t_start,t_end1;
    get_monotonic_boottime(&t_hr_start);

    do {
        get_monotonic_boottime(&t_hr_end1);
        if ( t_hr_end1.tv_nsec< t_hr_start.tv_nsec ) {
            t_hr_end1.tv_nsec=1000000000+t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        } else {
            t_hr_end1.tv_nsec=t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        }
        if ( t_hr_end1.tv_nsec>delay_ns ) break;
    } while(1);
}

//For non-timer-based busy check
void mvg_emmc_check_busy_and_reset(int delay_ms, int delay_us, u64 addr, u32 len)
{
    struct msdc_host *host=mtk_msdc_host[0];
    volatile unsigned int *reg1=(unsigned int *)(host->base+OFFSET_SDC_STS);
    //volatile unsigned int *reg2 = (unsigned int *)(host->base+OFFSET_MSDC_INT);
    static struct timespec t_start,t_end1;

    //Note: From msdc_response to here: 6571 consume 120us
    get_monotonic_boottime(&t_start);
    if ( delay_ms ) mdelay(delay_ms);
    if ( delay_us ) udelay(delay_us);
    get_monotonic_boottime(&t_end1);
    if ( (*reg1&SDC_STS_SDCBUSY) || (len==0) ) { //len=0 for erase
        //printk(KERN_ERR TAG " emmc_reset_mode %d\n", mvg_spoh_emmc_priv.emmc_reset_mode);
        if ( (mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF) ||
             (mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_WO_POWEROFF) ) {

            if ( t_end1.tv_nsec< t_start.tv_nsec ) {
                t_end1.tv_nsec=1000000000+t_end1.tv_nsec-t_start.tv_nsec;
            } else {
                t_end1.tv_nsec=t_end1.tv_nsec-t_start.tv_nsec;
            }

            if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF ) {
                hwPowerDown(MT6325_POWER_LDO_VEMC33, "msdc");
                //hwPowerDown(MT6323_POWER_LDO_VIO18, "msdc"); //VIO18 shall not be turned off since system will hang

                #if 0
                if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF ) {
                    while (upmu_get_qi_vemc_3v3_en());
                }
                #else
                mdelay(12); //wait LDO drop to about 0 --> Consult PMIC designer for time to wait.
                #endif
            }

            //Since print itself will add delay to perform wdt_arch_reset(),
            // only enable the next log only for measure planned timing and actual timing.
            if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF ) {
                //printk(KERN_ERR TAG " %d us reset with poweroff, test len %d, addr %x, plan %u us\n", t_end1.tv_nsec/1000, len, (unsigned int)addr, delay_ms*1000+delay_us);
            } else {
                //printk(KERN_ERR TAG " %d us reset without poweroff, test len %d, addr %x, plan %u us\n", t_end1.tv_nsec/1000, len, (unsigned int)addr, delay_ms*1000+delay_us);
            }

            mvg_wdt_reset();

        } else if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_NO_RESET ) {
            //printk(KERN_ERR TAG " mvg_emmc_check_busy_and_reset wait %u\n", delay);
            if ( t_end1.tv_nsec< t_start.tv_nsec ) {
                t_end1.tv_nsec=1000000000+t_end1.tv_nsec-t_start.tv_nsec;
            } else {
                t_end1.tv_nsec=t_end1.tv_nsec-t_start.tv_nsec;
            }
            printk(KERN_ERR TAG " not reset after %d us, test len %d , addr %x, plan %u us\n", ((int)t_end1.tv_nsec)/1000, len, (unsigned int)addr, delay_ms*1000+delay_us);
            //Not really reset --> for UT/simulation mode
            mvg_spoh_emmc_priv.reset_time_divisor=1;
            return;
        }

    } else {
        printk(KERN_ERR TAG " busy end before %d us, test len %u\n", delay_ms*1000+delay_us, mvg_spoh_emmc_priv.match_len);
        #if 0
        mvg_emmc_reset_planned=0;
        #else
        mvg_emmc_reset_timeout=1;
        mvg_emmc_reset_planned--;
        #endif
        mvg_spoh_emmc_priv.reset_delay_result=1;
        //Fail to reset/poweroff before program finish. Double time divisor to shorten time before reset/poweroff
        mvg_spoh_emmc_priv.reset_time_divisor=mvg_spoh_emmc_priv.reset_time_divisor<<1;
    }
}

int mvg_emmc_match(void *hostv, u64 addr, u32 opcode, u32 len)
{
    struct msdc_host *host=(struct msdc_host *)hostv;
    u32 time_max=0;
    u32 timer_inc=0, erase_address_matched=0, timer_inc2=0;

    if ( host->hw->host_function!= MSDC_EMMC ) return 0;

    if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_EXTERNAL_RANDOM_POWEROFF ) return 0;

    if ( mvg_emmc_reset_planned!=0 ) {
        mvg_emmc_reset_planned++;
    }
    if ( mvg_emmc_reset_planned==2 ) {
        //If we reach here, it means that this another command arrive msdc driver before excuting previously arranged timer function 
        #if 0
        //If we cancel the reset
        mvg_emmc_cancel_reset=1;
        mvg_emmc_reset_planned=0;
        //printk(KERN_ERR TAG " Cancel reset by opcode %u, addr %u, len %u, mvg_reset_scheduled %d\n", opcode, (u32)addr, len, mvg_reset_scheduled);
        #else
        //Wait until previously arranged timer function finish
        while( mvg_emmc_reset_planned==2 && mvg_emmc_reset_timeout==0 ) {
            //printk(KERN_ERR TAG "mvg_emmc_reset_planned2 %d, timeout %d\n", mvg_emmc_reset_planned, mvg_emmc_reset_timeout);
        }
        #endif
        
    }

    if ( mvg_reset_scheduled==1 )  return 0;

    //printk(KERN_ERR TAG " ERASE1 start %llx, end %llx\n", mvg_spoh_emmc_priv.erase_start,mvg_spoh_emmc_priv.erase_end);
    if (opcode==35) {
        mvg_spoh_emmc_priv.match_len=0;
        mvg_spoh_emmc_priv.erase_start=addr;
        mvg_spoh_emmc_priv.erase_set=1;
        //printk(KERN_ERR TAG " %s ERASE2 start %llx, end %llx\n", mvg_spoh_emmc_priv.erase_start,mvg_spoh_emmc_priv.erase_end);
        return 0;

    } else if ( (opcode==36) && (mvg_spoh_emmc_priv.erase_set==1) ) {
        mvg_spoh_emmc_priv.erase_end=addr;
        mvg_spoh_emmc_priv.erase_set=2;
        mvg_spoh_emmc_priv.match_len=((unsigned int)(mvg_spoh_emmc_priv.erase_end-mvg_spoh_emmc_priv.erase_start)+1)<<9;
        //printk(KERN_ERR TAG " ERASE3 start %llx, end %llx\n", mvg_spoh_emmc_priv.erase_start,mvg_spoh_emmc_priv.erase_end);
        return 0;

    } else if ( (opcode==38 ) && (mvg_spoh_emmc_priv.erase_set==2) ) {
        if ( mvg_addr_range_check(mvg_spoh_emmc_priv.erase_start) && mvg_addr_range_check(mvg_spoh_emmc_priv.erase_end) ) {
            erase_address_matched=1;
            mvg_spoh_emmc_priv.erase_set=0;
            len=mvg_spoh_emmc_priv.match_len;
            //printk(KERN_ERR TAG " ERASE MATCH len %u\n", len);
        }
    }

    if ( (erase_address_matched) || (mvg_addr_range_check(addr)) ) {
        if ( mvg_trigger() && mvg_get_wdt() ) {
            mvg_spoh_emmc_priv.reset_delay_result=0;

            mvg_spoh_emmc_priv.match_len=len;
            get_random_bytes(&timer_inc, sizeof(u32));
            //printk(KERN_ERR TAG "reset time mode %d\n",mvg_spoh_emmc_priv.emmc_reset_time_mode);
            //if ( mvg_spoh_emmc_priv.emmc_reset_time_mode==MVG_EMMC_RESET_LEN_DEPEND ) {
            if ( (mvg_spoh_emmc_priv.emmc_reset_time_mode==MVG_EMMC_RESET_LEN_DEPEND) || (erase_address_matched) ) {
                if ( erase_address_matched ) {
                    //time_max=w_dealy_max_tbl[mvg_spoh_emmc_priv.emmc_erase_group_sector_perf_table_idx];
                    time_max=100000000;
                } else if ( len<= 16*512 ) {
                    //For <=8K, use the longer value since flash programming time may vary in a wide range
                    time_max=w_dealy_max_tbl[4]; //16=2^4
                } else {
                    int i=5;
                    do {
                        if ( (((1<<i)*512)>=len) && (((1<<(i-1))*512)<len) ) {
                            time_max=w_dealy_max_tbl[i];
                            break;
                        }
                        i++;
                    } while (i<MVG_EMMC_PERFTABLE_SIZE);
                }
                if ( time_max==0 ) time_max=100000000;

                #if 0
                while ( mvg_spoh_emmc_priv.reset_time_divisor*50<time_max ) {
                    if ( mvg_spoh_emmc_priv.reset_time_divisor==1 ) break;
                    mvg_spoh_emmc_priv.reset_time_divisor=mvg_spoh_emmc_priv.reset_time_divisor>>1;
                }
                #else
                if ( mvg_spoh_emmc_priv.reset_time_divisor>time_max ) {
                    mvg_spoh_emmc_priv.reset_time_divisor=mvg_spoh_emmc_priv.reset_time_divisor>>2;
                }
                #endif
                time_max=time_max/mvg_spoh_emmc_priv.reset_time_divisor;
                if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF )
                    time_max+=TIME_POWER_DROPING;

                timer_inc=(timer_inc%time_max);
                
                if ( timer_inc==0 ) timer_inc=1;
                //printk(KERN_ERR TAG "len %d time_max %d, divisor %d\n",len, time_max, mvg_spoh_emmc_priv.reset_time_divisor);

            } else  {
                //time specified by application
                timer_inc=mvg_spoh_emmc_priv.emmc_reset_time_mode;
            }

            printk(KERN_ERR TAG " may reset in %d ns, test len %d , addr %x\n", timer_inc, len, (unsigned int)addr);

            #if defined(GPIO_TRIGGER_HW_POWER_LOSS)
            if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF ) {
                //printk(KERN_ERR "TIMER %u\n",timer_inc); //Temp
                if ( timer_inc<=TIME_TRIGGER_POWER_DROP_BEFORE_CMD ) {

                    if ( TIME_POWER_DROPING>timer_inc )
                        printk(KERN_ERR TAG " -%u ns plan poweroff  before CMD\n", timer_inc);
                    else
                        printk(KERN_ERR TAG " %u ns plan poweroff  after CMD\n", timer_inc-TIME_POWER_DROPING);

                    //Trigger GPIO
                    sdr_write32(0xF0014088,0x1); //disable pull of GPIO118
                    sdr_write32(0xF0005134,0x400000); //set output GPIO118 to 1
                    sdr_write32(0xF0005034,0x400000); //Set GPIO118 to output mode

                    if ( TIME_POWER_DROPING>timer_inc )
                        mvg_emmc_nanodelay(TIME_FROM_GPIO_TRIGGER_TO_POWER_DROP+timer_inc);
                    else
                        mvg_emmc_nanodelay(TIME_TRIGGER_POWER_DROP_BEFORE_CMD-timer_inc);

                    //printk(KERN_ERR TAG " wait %u ns before CMD, plan power off %u ns\n", t_hr_end1.tv_nsec, timer_inc);

                } else {
                    mvg_emmc_hrtimer_start(timer_inc-TIME_TRIGGER_POWER_DROP_BEFORE_CMD, addr, len);
                    timer_inc-=TIME_POWER_DROPING;
                }
            }
            #endif
            
            #if defined(GPIO_TRIGGER_HW_POWER_LOSS) || defined(SW_RESET_CONTROLLED_BY_TIMER)
            if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_WO_POWEROFF ) {
                //Use hr timer to trigger SW reset (watchdog reset) so that reset during cmd/response can occur
                mvg_emmc_hrtimer_start(timer_inc, addr, (opcode!=38)?len:0);
            }
            #endif

            mvg_emmc_reset_planned=1;
            return timer_inc;
        }
    }

    return 0;

}

int mvg_emmc_get_reset_time_divisor(void *power_loss_info)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;
    int ret;

    down_write(&(l_power_loss_info->rwsem));
    ret=priv->reset_time_divisor;
    up_write(&(l_power_loss_info->rwsem));
    return ret;
}

void mvg_emmc_set_reset_time_divisor(void *power_loss_info, int divisor)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;

    down_write(&(l_power_loss_info->rwsem));
    priv->reset_time_divisor=divisor;
    up_write(&(l_power_loss_info->rwsem));
}

void mvg_emmc_reset_mode_set(void *power_loss_info, int mode)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;

    down_write(&(l_power_loss_info->rwsem));
    if ( mode==MVG_EMMC_NO_RESET )
        priv->emmc_reset_mode=MVG_EMMC_NO_RESET;
    else if ( mode==MVG_EMMC_EXTERNAL_RANDOM_POWEROFF )
        priv->emmc_reset_mode=MVG_EMMC_EXTERNAL_RANDOM_POWEROFF;
    else if ( mode!=MVG_EMMC_RESET_WO_POWEROFF )
        priv->emmc_reset_mode=MVG_EMMC_RESET_W_POWEROFF;
    else
        priv->emmc_reset_mode=MVG_EMMC_RESET_WO_POWEROFF;
    up_write(&(l_power_loss_info->rwsem));
}

void mvg_emmc_reset_time_mode_set(void *power_loss_info, int mode)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;

    down_write(&(l_power_loss_info->rwsem));
    if ( mode>=1 ) //>=MVG_EMMC_RESET_USER_SPEC
        priv->emmc_reset_time_mode=mode*1000;
    else
        priv->emmc_reset_time_mode=mode;
    up_write(&(l_power_loss_info->rwsem));
}

void mvg_emmc_set_erase_group_sector(void *power_loss_info, int sector)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;
    int i=0;

    down_write(&(l_power_loss_info->rwsem));
    if ( sector>0 )
        priv->emmc_erase_group_sector=sector; //make it multiple of 512
    else
        priv->emmc_erase_group_sector=(1<<10); //Assume 512K bytes=2^10 sectors

    do {
        if ( (1<<i)==priv->emmc_erase_group_sector ) {
            priv->emmc_erase_group_sector_perf_table_idx=i;
            break;
        }
        i++;
    } while(i<MVG_EMMC_PERFTABLE_SIZE);

    up_write(&(l_power_loss_info->rwsem));
}

int mvg_emmc_get_delay_result(void *power_loss_info)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    struct mvg_spoh_emmc_priv_type *priv=(struct mvg_spoh_emmc_priv_type *)l_power_loss_info->drv_priv;
    int ret;

    down_write(&(l_power_loss_info->rwsem));
    ret=priv->reset_delay_result;
    //printk(KERN_ERR TAG "get_delay_result %d\n",ret);
    up_write(&(l_power_loss_info->rwsem));
    return ret;
}

int mvg_emmc_get_set_delay_table(void *power_loss_info, unsigned char *tbl)
{
    wdt_reboot_info *l_power_loss_info=(wdt_reboot_info *)power_loss_info;
    int ret;
    int i;

    down_write(&(l_power_loss_info->rwsem));
    ret  = copy_from_user(w_dealy_max_tbl, tbl, sizeof(w_dealy_max_tbl));

    for(i=0;i<MVG_EMMC_PERFTABLE_SIZE ;i++) {
        w_dealy_max_tbl[i]*=1000;
    }

    up_write(&(l_power_loss_info->rwsem));

    return ret;
}

enum hrtimer_restart mvg_emmc_hrtimer_check_busy_and_reset(struct hrtimer *timer)
{
    struct msdc_host *host=mtk_msdc_host[0];
    volatile unsigned int *reg1=(unsigned int *)(host->base+OFFSET_SDC_STS);
    //volatile unsigned int *reg2 = (unsigned int *)(host->base+OFFSET_MSDC_INT);
    static struct timespec t_start,t_end1;

    #if 0
    if ( mvg_emmc_cancel_reset ) {
        if ( mvg_reset_scheduled ) mvg_reset_scheduled=0;
        mvg_emmc_cancel_reset=0;
        goto out;
    }
    #endif

    if ( mvg_reset_scheduled ) {
        if ( *reg1&SDC_STS_SDCBUSY ) {
            printk(KERN_ERR TAG " busy\n");
            //Polling busy after 1ms
            hrtimer_start(&mvg_emmc_hrtimer, ktime_set(0, 1000000), HRTIMER_MODE_REL);
        } else{
            //printk(KERN_ERR TAG " not-busy\n");
            mvg_reset_scheduled=0;
        }

        goto out;
    }

    //if ( (*reg1&SDC_STS_SDCBUSY) || (hrtimer_len==0) ) { //len=0 means erase
    if ( (*reg1&SDC_STS_SDCBUSY) ) { //len=0 means erase
        //printk(KERN_ERR TAG " emmc_reset_mode %d\n", mvg_spoh_emmc_priv.emmc_reset_mode);
        //printk(KERN_ERR TAG "busy %d\n", (*reg1&SDC_STS_SDCBUSY));
        get_monotonic_boottime(&t_hr_end1);
        if ( t_hr_end1.tv_nsec< t_hr_start.tv_nsec ) {
            t_hr_end1.tv_nsec=1000000000+t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        } else {
            t_hr_end1.tv_nsec=t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        }
        
        if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF ) {
            t_hr_end1.tv_nsec+=TIME_FROM_GPIO_TRIGGER_TO_POWER_DROP;
            
            printk(KERN_ERR TAG " %d ns reset with poweroff, test len %d, addr %x, plan %u ns\n", (int)t_hr_end1.tv_nsec, hrtimer_len, (unsigned int)hrtimer_addr, hrtimer_delay_ns);
            //Trigger GPIO
            sdr_write32(0xF0014088,0x1); //disable pull of GPIO118
            sdr_write32(0xF0005134,0x400000); //set output GPIO118 to 1
            sdr_write32(0xF0005034,0x400000); //Set GPIO118 to output mode

            mvg_reset_scheduled=1;
            if ( hrtimer_len!=0 ) { // len=0 mean erase -> polling busy for non-erase
                //Polling busy after 1ms
                hrtimer_start(&mvg_emmc_hrtimer, ktime_set(0, 1000000), HRTIMER_MODE_REL);
            }

            goto out;

        } else if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_WO_POWEROFF ) {
            //Since print itself will add delay to perform wdt_arch_reset(),
            // only enable the next log only for measure planned timing and actual timing.
            //printk(KERN_ERR TAG " %d ns reset without poweroff, test len %d , addr %x, plan %u ns\n", (int)t_hr_end1.tv_nsec, hrtimer_len, (unsigned int)hrtimer_addr, hrtimer_delay_ns);
            mvg_wdt_reset();

        } else if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_NO_RESET ) {
            //printk(KERN_ERR TAG " mvg_emmc_check_busy_and_reset wait %u\n", delay);
            printk(KERN_ERR TAG " not reset after %d ns, test len %d , addr %x, plan %u ns\n", (int)t_hr_end1.tv_nsec, hrtimer_len, (unsigned int)hrtimer_addr, hrtimer_delay_ns);
            //Not really reset --> for UT/simulation mode
            mvg_spoh_emmc_priv.reset_time_divisor=1;
            goto out;
        }

    } else {
        get_monotonic_boottime(&t_hr_end1);
        if ( t_hr_end1.tv_nsec< t_hr_start.tv_nsec ) {
            t_hr_end1.tv_nsec=1000000000+t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        } else {
            t_hr_end1.tv_nsec=t_hr_end1.tv_nsec-t_hr_start.tv_nsec;
        }
        printk(KERN_ERR TAG " busy end after %d ns, test len %d , addr %x, plan %u ns\n", (int)t_hr_end1.tv_nsec, hrtimer_len, (unsigned int)hrtimer_addr, hrtimer_delay_ns);
        #if 0
        mvg_emmc_reset_planned=0;
        #else
        mvg_emmc_reset_timeout=1;
        mvg_emmc_reset_planned--;
        #endif
        mvg_spoh_emmc_priv.reset_delay_result=1;
        //Fail to reset/poweroff before program finish. Double time divisor to shorten time before reset/poweroff
        mvg_spoh_emmc_priv.reset_time_divisor=mvg_spoh_emmc_priv.reset_time_divisor<<1;
    }

out:
    return HRTIMER_NORESTART;
}

void mvg_emmc_hrtimer_start(int delay_ns, u64 addr, u32 len)
{
    ktime_t ktime_delay;

    //if ( delay_ms || delay_us ) {
        mvg_emmc_cancel_reset=0;
        hrtimer_delay_ns=delay_ns;
        hrtimer_len=len;
        hrtimer_addr=addr;
        ktime_delay=ktime_set(0, hrtimer_delay_ns);
        if ( mvg_spoh_emmc_priv.emmc_reset_mode==MVG_EMMC_RESET_W_POWEROFF )
            hrtimer_delay_ns+=TIME_FROM_GPIO_TRIGGER_TO_POWER_DROP;
        get_monotonic_boottime(&t_hr_start);
        mvg_emmc_reset_timeout=0;
        hrtimer_start(&mvg_emmc_hrtimer, ktime_delay, HRTIMER_MODE_REL);
    //}
}

void mvg_emmc_hrtimer_init(unsigned long data)
{
    hrtimer_init(&mvg_emmc_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mvg_emmc_hrtimer.function=mvg_emmc_hrtimer_check_busy_and_reset;
}

#endif


MODULE_AUTHOR("light.hsieh <light.hsieh@mediatek.com>");
MODULE_DESCRIPTION(" This module is for power loss test");

