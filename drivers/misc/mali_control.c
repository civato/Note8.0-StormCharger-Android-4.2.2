/* drivers/misc/mali_control.c
 *
 * based on Michael Wodkins's gpu_control implementation on S2
 * a little modified to have clock and threshold controls by gokhanmoral
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of the GNU General Public License as published by the
 *  Free Software Foundation;
 */

#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/kallsyms.h>

#define GPU_MAX_CLOCK 700
#define GPU_MIN_CLOCK 10

#define MIN_VOLTAGE_GPU  600000
#define MAX_VOLTAGE_GPU 1400000

typedef struct mali_dvfs_tableTag{
    unsigned int clock;
    unsigned int freq;
    unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
} mali_dvfs_threshold_table;

typedef struct mali_runtime_resumeTag{
	int clk;
	int vol;
} mali_runtime_resume_table;

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

extern mali_runtime_resume_table mali_runtime_resume;
extern mali_dvfs_table mali_dvfs[5];
extern mali_dvfs_threshold_table mali_dvfs_threshold[5];
extern mali_dvfs_staycount_table mali_dvfs_staycount[5];


static ssize_t gpu_clock_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "Step0: %d\nStep1: %d\nStep2: %d\nStep3: %d\n"
						"Threshold0-1/up-down: %d%% %d%%\n"
						"Threshold1-2/up-down: %d%% %d%%\n"
						"Threshold2-3/up-down: %d%% %d%%\n",
		mali_dvfs[0].clock, mali_dvfs[1].clock, mali_dvfs[2].clock, mali_dvfs[3].clock,
		mali_dvfs_threshold[0].upthreshold, 
		mali_dvfs_threshold[1].downthreshold,
		mali_dvfs_threshold[1].upthreshold,
		mali_dvfs_threshold[2].downthreshold,
		mali_dvfs_threshold[2].upthreshold,
		mali_dvfs_threshold[3].downthreshold
		);
}


static ssize_t gpu_clock_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int ret = -EINVAL;
	int i = 0;
	unsigned int g[8];

	if ( (ret=sscanf(buf, "%d%% %d%% %d%% %d%% %d%% %d%%",
			&g[0], &g[1], &g[2], &g[3], &g[4], &g[5])) == 6 )
	{
		if(g[1]<0 || g[0]>100) return -EINVAL;
		mali_dvfs_threshold[0].upthreshold = g[0];
		mali_dvfs_threshold[1].downthreshold = g[1];
		mali_dvfs_threshold[1].upthreshold = g[2];
		mali_dvfs_threshold[2].downthreshold = g[3];
		mali_dvfs_threshold[2].upthreshold = g[4];
		mali_dvfs_threshold[3].downthreshold = g[5];
	} 
	else {
	  if ( (ret=sscanf(buf, "%d %d %d %d", &g[0], &g[1], &g[2], &g[3]))!=5 )
			return -EINVAL;
		/* safety floor and ceiling - netarchy */
		for( i = 0; i < 5; i++ ) {
			if (g[i] < GPU_MIN_CLOCK) {
				g[i] = GPU_MIN_CLOCK;
			}
			else if (g[i] > GPU_MAX_CLOCK) {
				g[i] = GPU_MAX_CLOCK;
			}
			mali_dvfs[i].clock=g[i];
		}
	}
	return count;	
}

static ssize_t gpu_staycount_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d %d %d %d\n", 
	mali_dvfs_staycount[0].staycount,
	mali_dvfs_staycount[1].staycount,
	mali_dvfs_staycount[2].staycount,
	mali_dvfs_staycount[3].staycount
	);
}

static ssize_t gpu_staycount_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int ret = -EINVAL;
	int i1, i2, i3, i4;

    if ( (ret=sscanf(buf, "%d %d %d %d", &i1, &i2, &i3, &i4))!=4 )
		return -EINVAL;
	mali_dvfs_staycount[0].staycount = i1;
	mali_dvfs_staycount[1].staycount = i2;
	mali_dvfs_staycount[2].staycount = i3;
	mali_dvfs_staycount[3].staycount = i4;
	return count;	
}

static ssize_t gpu_voltage_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "Step1: %d\nStep2: %d\nStep3: %d\nStep4: %d\n",
			mali_dvfs[0].vol, mali_dvfs[1].vol,mali_dvfs[2].vol,mali_dvfs[3].vol);
}

static ssize_t gpu_voltage_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int ret = -EINVAL;
	int i = 0;
	unsigned int gv[5];

	ret = sscanf(buf, "%d %d %d %d", &gv[0], &gv[1], &gv[2], &gv[3]);
	if(ret!=4) 
	{
		return -EINVAL;
	}
	
    /* safety floor and ceiling - netarchy */
    for( i = 0; i < 4; i++ ) {
        if (gv[i] < MIN_VOLTAGE_GPU) {
            gv[i] = MIN_VOLTAGE_GPU;
        }
        else if (gv[i] > MAX_VOLTAGE_GPU) {
            gv[i] = MAX_VOLTAGE_GPU;
    	}
		mali_dvfs[i].vol=gv[i];
    }
//	mali_runtime_resume.vol = mali_dvfs[1].vol;
	return count;	
}

static DEVICE_ATTR(voltage_control, S_IRUGO | S_IWUGO, gpu_voltage_show, gpu_voltage_store);
static DEVICE_ATTR(clock_control, S_IRUGO | S_IWUGO, gpu_clock_show, gpu_clock_store);
static DEVICE_ATTR(staycount_control, S_IRUGO | S_IWUGO, gpu_staycount_show, gpu_staycount_store);

static struct attribute *malicontrol_attributes[] = {
	&dev_attr_voltage_control.attr,
	&dev_attr_clock_control.attr,
	&dev_attr_staycount_control.attr,
	NULL
};

static struct attribute_group malicontrol_group = {
	.attrs = malicontrol_attributes,
};

static struct miscdevice malicontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mali_control",
};

static int __init malicontrol_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, malicontrol_device.name);

    ret = misc_register(&malicontrol_device);
    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, malicontrol_device.name);
	    return 1;
	}
    if (sysfs_create_group(&malicontrol_device.this_device->kobj, &malicontrol_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", malicontrol_device.name);
	}

    return 0;
}

static void __exit malicontrol_exit(void)
{
	sysfs_remove_group(&malicontrol_device.this_device->kobj, &malicontrol_group);
	misc_deregister(&malicontrol_device);
}

module_init( malicontrol_init );
module_exit( malicontrol_exit );

MODULE_AUTHOR("Gokhan Moral <gm@alumni.bilkent.edu.tr>");
MODULE_DESCRIPTION("Mali over/under clocking/volting module");
MODULE_LICENSE("GPL");

