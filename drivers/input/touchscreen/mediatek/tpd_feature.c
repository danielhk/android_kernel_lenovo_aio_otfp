#include "tpd.h"

/* lenovo liuyw2 add
 *
 * some feature, such as fw version info,glove,gesture
*/
static struct tpd_version_info *tpd_info = NULL;

static struct fwu_userspace_t  fwu_userspace = {
	.lastest_fwid = NULL,
	.upgrade_progress = NULL,
};
static struct tpd_glove_t *glove_func = NULL;
static struct tpd_gesture_t *gesture_func;
static int tpd_ic_ready = 0;


static ssize_t le_tpd_info_show(struct kobject *kobj, 
					struct kobj_attribute *attr, char *buf)
{
	if (tpd_info)
		return sprintf(buf,"name=%s; types=%d; fw_num=0x%08x;\n",
				tpd_info->name, 
				tpd_info->types, 
				tpd_info->fw_num);
	else
		return sprintf(buf,"name=%s; types=%d; fw_num=0x%08x;\n",
				"Not set",
				0,
				521);
}

static struct kobj_attribute le_tpd_info_attr = {
    .attr = {
        .name = "lenovo_tpd_info",
        .mode = S_IRUGO,
    },
    .show = &le_tpd_info_show,
};

void le_tpd_reg_feature_tpd_info(struct tpd_version_info *info)
{
	tpd_info = info;
}

/* lastest fwid */
static ssize_t le_tpd_lastest_fwid_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (fwu_userspace.upgrade_progress)
 		return sprintf(buf,"0x%08x\n", fwu_userspace.lastest_fwid());
 	else
		return sprintf(buf,"%d\n", 0);
}

static struct kobj_attribute le_tpd_lastest_fwid_attr = {
    .attr = {
        .name = "lenovo_tpd_lastest_fwid",
        .mode = S_IRUGO,
    },
    .show = &le_tpd_lastest_fwid_show,
};

/* tpd upgrade status */
static ssize_t le_tpd_upgrade_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (fwu_userspace.upgrade_progress)
 		return sprintf(buf,"%d\n", fwu_userspace.upgrade_progress());
 	else
		return sprintf(buf,"%d\n", 0);
}

static struct kobj_attribute le_tpd_upgrade_status_attr = {
    .attr = {
        .name = "lenovo_tpd_upgrade_status",
        .mode = S_IRUGO,
    },
    .show = &le_tpd_upgrade_status_show,
};

void le_tpd_reg_feature_fwu_userspace_cb(int (*d)(void), int (*p)(void))
{
	fwu_userspace.lastest_fwid = d;
	fwu_userspace.upgrade_progress = p;
}

static ssize_t le_tpd_gesture_letter_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
	if (gesture_func)
		return sprintf(buf,"%d\n", gesture_func->letter);
	else
		return sprintf(buf,"%d\n", -1);
}

static struct kobj_attribute le_tpd_gesture_letter__attr = {
    .attr = {
        .name = "lenovo_flag",
        .mode = S_IRUGO,
    },
    .show = &le_tpd_gesture_letter_show,
};

static ssize_t le_tpd_gesture_wakeup_enable_show(struct kobject *kobj, 
							struct kobj_attribute *attr, char *buf)
{
	if (gesture_func)
		return sprintf(buf,"%d.\n", gesture_func->wakeup_enable);
	else
		return sprintf(buf,"%d.\n", -1);
}

static ssize_t le_tpd_gesture_wakeup_enable_store(struct kobject *kobj, 
							struct kobj_attribute *attr, const char *buf, size_t size)
{
	int mode;
	int res = sscanf(buf, "%d", &mode);
	if (res != 1) {
		printk("%s: [wj]expect 1 numbers\n", __func__);
	} else {
		if (gesture_func)
			gesture_func->wakeup_enable = mode;
		else
			printk("%s: gesture_wakeup_enable is null\n", __func__);
	}
	return size;
}

void le_tpd_reg_feature_gesture_func(struct  tpd_gesture_t  *ge)
{
	gesture_func = ge;
}

static struct kobj_attribute le_tpd_gesture_wakeup_enable_attr = {
	.attr = {
		.name = "tpd_suspend_status",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = &le_tpd_gesture_wakeup_enable_show,
	.store = &le_tpd_gesture_wakeup_enable_store,
};

static void set_tpd_glove_status(int mode)
{
	if (glove_func) {
		glove_func->pre_status = mode? 1 : 0;
		if (glove_func->set_mode)
			glove_func->set_mode(mode);
	} else
		printk("%s dont support glove func\n", __func__);
}

static int get_tpd_glove_status(void)
{
	if (glove_func && glove_func->get_mode)
		return glove_func->get_mode();
	else {
		return -1;
		printk("%s dont support glove func\n", __func__);
	}
}

static ssize_t lenovo_tpd_glove_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d.\n", get_tpd_glove_status());	
}

static ssize_t lenovo_tpd_glove_store(struct kobject *kobj, struct kobj_attribute *attr, 
					const char *buf, size_t size)	
{
	int mode;
	int res = sscanf(buf, "%d", &mode);
	if (res != 1) {
		printk("%s: [wj]expect 1 numbers\n", __func__);
	} else {	
		set_tpd_glove_status(mode);
	}
	return size;
}

static struct kobj_attribute le_tpd_glove_attr = {
	.attr = {
		.name = "tpd_glove_status",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = &lenovo_tpd_glove_show,
	.store = &lenovo_tpd_glove_store,
};

void le_tpd_reg_feature_glove_func(struct tpd_glove_t *gf)
{
	glove_func = gf;
}

static int glove_usb_in_set = 0;
void le_tpd_glove_usb_notify(unsigned int st)
{
	if (!glove_func)
		return;

	if (st) {
		glove_func->usb_st = 1;
		if ((glove_usb_in_set == 0) && (glove_func->pre_status == 1)) {
			printk("tpd feature usb in set glove function disabel.\n");
			if (glove_func->set_mode)
				glove_func->set_mode(0);
			glove_usb_in_set = 1;
		}
	} else {
		if((glove_func->usb_st == 1) && (glove_func->pre_status == 1)) {
			printk("tpd feature usb in set glove function enable.\n");
			if (glove_func->set_mode)
				glove_func->set_mode(1);
		}
		glove_func->usb_st = 0;
		glove_usb_in_set = 0;
	}
}

/* attrs and the group */
static struct attribute *properties_attrs[] = {
	&le_tpd_info_attr.attr,
	&le_tpd_lastest_fwid_attr.attr,
	&le_tpd_upgrade_status_attr.attr,
	&le_tpd_gesture_letter__attr.attr,
	&le_tpd_gesture_wakeup_enable_attr.attr,
	&le_tpd_glove_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

int le_add_feature_attrs(struct kobject *kobj)
{
	return sysfs_create_group(kobj, &properties_attr_group);
}


int tpd_ic_ready_get(void)
{
	return tpd_ic_ready;
}

void tpd_ic_ready_set(int rd)
{
	tpd_ic_ready = rd;
}