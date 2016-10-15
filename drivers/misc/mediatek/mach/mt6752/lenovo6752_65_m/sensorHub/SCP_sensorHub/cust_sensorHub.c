#include <linux/types.h>
#include <cust_sensorHub.h>

static struct sensorHub_hw cust_sensorHub_hw = {
    .is_batch_enabled = true,
};
struct sensorHub_hw *get_cust_sensorHub_hw(void) {
    return &cust_sensorHub_hw;
}

