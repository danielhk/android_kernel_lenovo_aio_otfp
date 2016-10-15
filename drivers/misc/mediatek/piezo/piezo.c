/*
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/platform_device.h>

#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>

#include <linux/timer.h>
#include <linux/delay.h>




#define EINT_PIN_PIEZO_OUT 	(0)
#define EINT_PIN_PIEZO_IN 	(1)

int piezo_cur_eint_state = EINT_PIN_PIEZO_OUT;
int lenovo_piezo_irq;
unsigned int piezo_gpiopin,piezodebounce;
static int g_piezo_first = 1;
int piezo_shell_status = -1;
extern bool piezo_working_flag;

static struct switch_dev piezo_switch_dev;

static struct work_struct piezo_eint_work;
static struct workqueue_struct * piezo_eint_workqueue = NULL;


static void piezo_eint_work_callback(struct work_struct *work)
{
	int state = mt_get_gpio_in(GPIO_LENOVO_PIEZO_EINT_PIN);
	if(piezo_cur_eint_state == EINT_PIN_PIEZO_IN)
	{
		piezo_shell_status= 1;
		if(piezo_working_flag == true && state == 0){
			mt_set_gpio_mode(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_MODE_00); //GPIO70: DPI_D3, mode 0
			mt_set_gpio_pull_enable(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_dir(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_DIR_OUT); // output
			mt_set_gpio_out(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_OUT_ONE); // high enable
			
			mt_set_gpio_mode(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_MODE_00); //GPIO132: DPI_D3, mode 0
        		mt_set_gpio_pull_enable(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_PULL_ENABLE);
        		mt_set_gpio_dir(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_DIR_OUT); // output
			mt_set_gpio_out(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_OUT_ONE); // high enable
		}
	}
	else
	{
		piezo_shell_status= 0;
		mt_set_gpio_mode(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_MODE_00); //GPIO70: DPI_D3, mode 0
		mt_set_gpio_dir(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_DIR_OUT); // output
		mt_set_gpio_out(GPIO_LENOVO_PIEZO_AUDPWR_ON_ENABLE_PIN, GPIO_OUT_ZERO); // low disbale

		mt_set_gpio_mode(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_MODE_00); //GPIO132: DPI_D3, mode 0
		mt_set_gpio_dir(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_DIR_OUT); // output
		mt_set_gpio_out(GPIO_LENOVO_PIEZO_ENABLE_PIN, GPIO_OUT_ZERO); // low disbale
	}
	switch_set_state(&piezo_switch_dev, piezo_cur_eint_state);
	printk("piezo_cur_eint_state = %d \n",piezo_cur_eint_state);
}

static irqreturn_t piezo_cover_eint_handler(int irq, void *data)
{
	int ret = 0;
	
	if(piezo_cur_eint_state == EINT_PIN_PIEZO_IN){
		if (CUST_EINT_LENOVO_PIEZO_EINT_TYPE == CUST_EINTF_TRIGGER_HIGH){
					mt_eint_set_polarity(CUST_EINT_LENOVO_PIEZO_EINT_NUM, (1));
		}else{
					mt_eint_set_polarity(CUST_EINT_LENOVO_PIEZO_EINT_NUM, (0));
		}
		mt_eint_set_hw_debounce(CUST_EINT_LENOVO_PIEZO_EINT_NUM, CUST_EINT_LENOVO_PIEZO_EINT_DEBOUNCE_CN);
		piezo_cur_eint_state = EINT_PIN_PIEZO_OUT;
	}
	else{
		if (CUST_EINT_LENOVO_PIEZO_EINT_TYPE == CUST_EINTF_TRIGGER_HIGH){
					mt_eint_set_polarity(CUST_EINT_LENOVO_PIEZO_EINT_NUM, !(1));
		}else{
					mt_eint_set_polarity(CUST_EINT_LENOVO_PIEZO_EINT_NUM, !(0));
		}
		mt_eint_set_hw_debounce(CUST_EINT_LENOVO_PIEZO_EINT_NUM, CUST_EINT_LENOVO_PIEZO_EINT_DEBOUNCE_CN);
		piezo_cur_eint_state = EINT_PIN_PIEZO_IN;
	}
	//enable_irq(lenovo_piezo_irq);
	printk("%s \n", __func__);
	ret = queue_work(piezo_eint_workqueue, &piezo_eint_work);
	return IRQ_HANDLED;

}

static inline int piezo_setup_eint(void)
{
	int ret;
	u32 ints[2]={0,0};
	struct device_node *node;

	mt_set_gpio_dir(GPIO_LENOVO_PIEZO_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_LENOVO_PIEZO_EINT_PIN, GPIO_LENOVO_PIEZO_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_LENOVO_PIEZO_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_LENOVO_PIEZO_EINT_PIN, GPIO_PULL_UP);

	node = of_find_compatible_node(NULL,NULL,"mediatek, LENOVO_PIEZO_EINT-eint");		
	if(node){
		of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
		#if 0
		piezo_gpiopin = ints[0];
		piezodebounce = ints[1];
		mt_gpio_set_debounce(piezo_gpiopin,piezodebounce);
		#else
		mt_eint_set_hw_debounce(CUST_EINT_LENOVO_PIEZO_EINT_NUM, CUST_EINT_LENOVO_PIEZO_EINT_DEBOUNCE_CN);
		#endif
		lenovo_piezo_irq = irq_of_parse_and_map(node,0);
		ret=request_irq(lenovo_piezo_irq, piezo_cover_eint_handler, IRQF_TRIGGER_LOW, "piezo-eint", NULL);
		if(ret>0)
			printk("[Ceramic_speaker]EINT IRQ  AVAILABLE\n");	
		}
	//disable_irq(lenovo_piezo_irq);
	return 0;
}


static int piezo_probe(struct platform_device *dev)
{
	int state;
	pr_info("%s\n", __func__);
		
	piezo_switch_dev.name = "piezo";
	piezo_switch_dev.index = 0;
	piezo_switch_dev.state = -1;
	switch_dev_register(&piezo_switch_dev);

	//state = !mt_get_gpio_in(GPIO_LENOVO_PIEZO_EINT_PIN);
	//switch_set_state(&piezo_switch_dev, state);

       
	//if(g_piezo_first ==1){
	piezo_eint_workqueue = create_singlethread_workqueue("piezo_eint");
	INIT_WORK(&piezo_eint_work, piezo_eint_work_callback);
	piezo_setup_eint();
		//g_piezo_first = 0;
		//}

	printk("%s done, state=%d\n", __func__, state);
	return 0;
}

static int piezo_remove(struct platform_device *dev)
{
	printk("%s\n", __func__);
	cancel_work_sync(&piezo_eint_work);
	destroy_workqueue(piezo_eint_workqueue);
	switch_dev_unregister(&piezo_switch_dev);
	return 0;
}

static struct platform_device piezo_device = {
	.name = "piezo-speaker",
	.id   = -1,
};

static struct platform_driver piezo_driver = {
	.probe      = piezo_probe,
	.remove     = piezo_remove,
	.driver     = {
		.name   = "piezo-speaker",
	},
};

static int __init piezo_init(void)
{
	int ret;
	printk("%s\n", __func__);

	ret = platform_device_register(&piezo_device);
	if(ret){
		pr_err("fail to register piezo device (%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&piezo_driver);
	if(ret){
		pr_err("fail to register piezo driver (%d)\n", ret);
		return ret;
	}

	printk("%s done\n", __func__);
	return 0;
}

static void __exit piezo_exit(void)
{
    printk("%s\n", __func__);
    platform_driver_unregister(&piezo_driver);
    platform_device_unregister(&piezo_device);
}


module_init(piezo_init);
module_exit(piezo_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("Lenovo piezo speaker Driver");
MODULE_LICENSE("GPL");

