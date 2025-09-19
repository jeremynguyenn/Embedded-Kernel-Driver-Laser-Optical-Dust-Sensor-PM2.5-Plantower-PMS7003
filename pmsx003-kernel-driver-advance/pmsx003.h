#ifndef PMSX003_H
#define PMSX003_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/ioctl.h>
#include <linux/termios.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/serial_core.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mqueue.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/shm.h>

/* Chip definitions */
#define CHIP_NAME "PLANTOWER PMSX003"
#define MANUFACTURER_NAME "PLANTOWER"
#define SUPPLY_VOLTAGE_MIN 4.5f
#define SUPPLY_VOLTAGE_MAX 5.5f
#define MAX_CURRENT 100.0f
#define TEMPERATURE_MIN -10.0f
#define TEMPERATURE_MAX 60.0f
#define DRIVER_VERSION 1001

/* Command definitions */
#define PMSX003_COMMAND_READ 0xE2
#define PMSX003_COMMAND_CHANGE_WORKING_MODE 0xE1
#define PMSX003_COMMAND_CHANGE_CHIP_MODE 0xE4

/* Hard mode enumeration */
typedef enum {
    PMSX003_HARD_MODE_SLEEP = 0x00,
    PMSX003_HARD_MODE_NORMAL = 0x01,
} pmsx003_hard_mode_t;

/* Mode enumeration */
typedef enum {
    PMSX003_MODE_PASSIVE = 0x00,
    PMSX003_MODE_ACTIVE = 0x01,
} pmsx003_mode_t;

/* Data structure definition */
typedef struct pmsx003_data_s {
    uint16_t pm1p0_standard_ug_m3;
    uint16_t pm2p5_standard_ug_m3;
    uint16_t pm10_standard_ug_m3;
    uint16_t pm1p0_atmospheric_ug_m3;
    uint16_t pm2p5_atmospheric_ug_m3;
    uint16_t pm10_atmospheric_ug_m3;
    uint16_t beyond_0p3um;
    uint16_t beyond_0p5um;
    uint16_t beyond_1p0um;
    uint16_t beyond_2p5um;
    uint16_t beyond_5p0um;
    uint16_t beyond_10um;
    uint8_t version;
    uint8_t error_code;
} pmsx003_data_t;

/* Info structure */
typedef struct pmsx003_info_s {
    char chip_name[32];
    char manufacturer_name[32];
    char interface[8];
    float supply_voltage_min_v;
    float supply_voltage_max_v;
    float max_current_ma;
    float temperature_min;
    float temperature_max;
    uint32_t driver_version;
} pmsx003_info_t;

/* Handle structure definition */
struct pmsx003_dev {
    struct device *dev;
    struct mutex lock;
    struct workqueue_struct *wq;
    struct delayed_work read_work;
    struct uart_driver uart_drv;
    struct uart_port *uart_port;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *set_gpio;
    uint8_t mode;
    uint8_t inited;
    pmsx003_data_t data;
    unsigned long periodic_delay;
    dev_t devno;
    struct cdev cdev;
    struct class *class;
    struct attribute_group attrs;
    struct dentry *debugfs_dir;
    struct completion data_ready;
    struct semaphore data_sem;
    pid_t user_pid;
    struct task_struct *reader_thread;
    atomic_t open_count;
    wait_queue_head_t read_queue;
    struct mqueue_inode_info *mq;
    void *shm_addr;
};

/* IOCTL commands */
#define PMSX003_MAGIC 'P'
#define PMSX003_IOCTL_SET_MODE _IOW(PMSX003_MAGIC, 1, int)
#define PMSX003_IOCTL_SET_HARD_MODE _IOW(PMSX003_MAGIC, 2, int)
#define PMSX003_IOCTL_SLEEP _IO(PMSX003_MAGIC, 3)
#define PMSX003_IOCTL_WAKE_UP _IO(PMSX003_MAGIC, 4)
#define PMSX003_IOCTL_RESET _IO(PMSX003_MAGIC, 5)
#define PMSX003_IOCTL_START_PERIODIC_READ _IOW(PMSX003_MAGIC, 6, unsigned long)
#define PMSX003_IOCTL_STOP_PERIODIC_READ _IO(PMSX003_MAGIC, 7)
#define PMSX003_IOCTL_GET_INFO _IOR(PMSX003_MAGIC, 8, pmsx003_info_t)
#define PMSX003_IOCTL_SET_NONBLOCK _IOW(PMSX003_MAGIC, 9, int)

#endif /* PMSX003_H */