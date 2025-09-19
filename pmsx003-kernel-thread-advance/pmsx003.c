#include "pmsx003.h"
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/shm.h>
#include <linux/atomic.h>
#include <linux/smp.h>

static struct uart_driver pmsx003_uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = "pmsx003_uart",
    .dev_name = "ttyPMS",
    .nr = PMSX003_UART_PORTS,
};

static int pmsx003_uart_startup(struct uart_port *port) {
    struct pmsx003_dev *dev = container_of(port, struct pmsx003_dev, uart_port[0]);
    dev_info(dev->dev, "Starting UART for PMSX003\n");
    return 0;
}

static void pmsx003_uart_shutdown(struct uart_port *port) {
    struct pmsx003_dev *dev = container_of(port, struct pmsx003_dev, uart_port[0]);
    dev_info(dev->dev, "Shutting down UART for PMSX003\n");
}

static void pmsx003_uart_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old) {
    termios->c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    termios->c_iflag = IGNPAR;
    termios->c_oflag = 0;
    termios->c_lflag = 0;
}

static const struct uart_ops pmsx003_uart_ops = {
    .tx_empty = NULL,
    .set_mctrl = NULL,
    .get_mctrl = NULL,
    .stop_tx = NULL,
    .start_tx = NULL,
    .stop_rx = NULL,
    .enable_ms = NULL,
    .break_ctl = NULL,
    .startup = pmsx003_uart_startup,
    .shutdown = pmsx003_uart_shutdown,
    .set_termios = pmsx003_uart_set_termios,
    .type = NULL,
    .release_port = NULL,
    .request_port = NULL,
    .config_port = NULL,
    .verify_port = NULL,
    .pm = NULL,
};

static uint8_t a_pmsx003_make_frame(uint8_t command, uint8_t data, uint8_t output[7]) {
    uint8_t i;
    uint16_t lrc = 0;

    output[0] = 0x42;
    output[1] = 0x4D;
    output[2] = command;
    output[3] = 0x00;
    output[4] = data;

    for (i = 0; i < 5; i++) {
        lrc += output[i];
    }
    output[5] = (lrc >> 8) & 0xFF;
    output[6] = lrc & 0xFF;

    return 0;
}

static uint8_t a_pmsx003_parse_frame(struct pmsx003_dev *dev, uint8_t input[8], uint8_t command, uint8_t *data) {
    uint8_t i;
    uint16_t lrc = 0, lrc_check;

    for (i = 0; i < 6; i++) lrc += input[i];
    lrc_check = ((uint16_t)input[6] << 8) | input[7];
    if (lrc != lrc_check) {
        dev_err(dev->dev, "pmsx003: LRC check error\n");
        return 1;
    }
    if (input[0] != 0x42 || input[1] != 0x4D) {
        dev_err(dev->dev, "pmsx003: Invalid frame header\n");
        return 1;
    }

    uint16_t len = ((uint16_t)input[2] << 8) | input[3];
    if (len != 4) {
        dev_err(dev->dev, "pmsx003: Invalid frame length\n");
        return 1;
    }
    if (input[4] != command) {
        dev_err(dev->dev, "pmsx003: Invalid command\n");
        return 1;
    }
    *data = input[5];

    return 0;
}

static uint8_t a_pmsx003_parse_data(struct pmsx003_dev *dev, uint8_t input[32], pmsx003_data_t *data) {
    uint8_t i;
    uint16_t lrc = 0, lrc_check;

    for (i = 0; i < 30; i++) lrc += input[i];
    lrc_check = ((uint16_t)input[30] << 8) | input[31];
    if (lrc != lrc_check) {
        dev_err(dev->dev, "pmsx003: LRC check error\n");
        return 1;
    }
    if (input[0] != 0x42 || input[1] != 0x4D) {
        dev_err(dev->dev, "pmsx003: Invalid frame header\n");
        return 1;
    }

    uint16_t len = ((uint16_t)input[2] << 8) | input[3];
    if (len != 28) {
        dev_err(dev->dev, "pmsx003: Invalid data frame length\n");
        return 1;
    }

    data->pm1p0_standard_ug_m3 = ((uint16_t)input[4] << 8) | input[5];
    data->pm2p5_standard_ug_m3 = ((uint16_t)input[6] << 8) | input[7];
    data->pm10_standard_ug_m3 = ((uint16_t)input[8] << 8) | input[9];
    data->pm1p0_atmospheric_ug_m3 = ((uint16_t)input[10] << 8) | input[11];
    data->pm2p5_atmospheric_ug_m3 = ((uint16_t)input[12] << 8) | input[13];
    data->pm10_atmospheric_ug_m3 = ((uint16_t)input[14] << 8) | input[15];
    data->beyond_0p3um = ((uint16_t)input[16] << 8) | input[17];
    data->beyond_0p5um = ((uint16_t)input[18] << 8) | input[19];
    data->beyond_1p0um = ((uint16_t)input[20] << 8) | input[21];
    data->beyond_2p5um = ((uint16_t)input[22] << 8) | input[23];
    data->beyond_5p0um = ((uint16_t)input[24] << 8) | input[25];
    data->beyond_10um = ((uint16_t)input[26] << 8) | input[27];
    data->version = input[28];
    data->error_code = input[29];

    return 0;
}

static uint8_t pmsx003_uart_write(struct pmsx003_dev *dev, uint8_t *buf, uint16_t len) {
    int ret = uart_write(dev->uart_port, buf, len);
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: UART write failed: %d\n", ret);
        return 1;
    }
    return 0;
}

static uint16_t pmsx003_uart_read(struct pmsx003_dev *dev, uint8_t *buf, uint16_t len) {
    int ret = uart_read(dev->uart_port, buf, len);
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: UART read failed: %d\n", ret);
        return 0;
    }
    return (uint16_t)ret;
}

static uint8_t pmsx003_uart_flush(struct pmsx003_dev *dev) {
    int ret = uart_flush_tx(dev->uart_port);
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: UART flush failed: %d\n", ret);
        return 1;
    }
    return 0;
}

static int pmsx003_init_uart(struct pmsx003_dev *dev) {
    int ret;

    pmsx003_uart_driver.owner = THIS_MODULE;
    ret = uart_register_driver(&pmsx003_uart_driver);
    if (ret) {
        dev_err(dev->dev, "pmsx003: UART driver registration failed: %d\n", ret);
        return ret;
    }

    dev->uart_port->ops = &pmsx003_uart_ops;
    ret = uart_add_one_port(&pmsx003_uart_driver, dev->uart_port);
    if (ret) {
        dev_err(dev->dev, "pmsx003: UART port add failed: %d\n", ret);
        uart_unregister_driver(&pmsx003_uart_driver);
        return ret;
    }

    return 0;
}

static void pmsx003_deinit_uart(struct pmsx003_dev *dev) {
    uart_remove_one_port(&pmsx003_uart_driver, dev->uart_port);
    uart_unregister_driver(&pmsx003_uart_driver);
}

static void pmsx003_timer_callback(struct timer_list *t) {
    struct pmsx003_dev *dev = from_timer(dev, t, periodic_timer);
    queue_delayed_work(dev->wq, &dev->read_work, 0);
    if (dev->periodic_delay) {
        mod_timer(&dev->periodic_timer, jiffies + msecs_to_jiffies(dev->periodic_delay));
    }
}

static ssize_t show_version(struct device *ddev, struct device_attribute *attr, char *buf) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    down_read(&dev->rwlock);
    ssize_t ret = scnprintf(buf, PAGE_SIZE, "%u\n", dev->data.version);
    up_read(&dev->rwlock);
    return ret;
}

static ssize_t show_error_code(struct device *ddev, struct device_attribute *attr, char *buf) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    down_read(&dev->rwlock);
    ssize_t ret = scnprintf(buf, PAGE_SIZE, "%u\n", dev->data.error_code);
    up_read(&dev->rwlock);
    return ret;
}

static ssize_t show_pm2p5(struct device *ddev, struct device_attribute *attr, char *buf) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    down_read(&dev->rwlock);
    ssize_t ret = scnprintf(buf, PAGE_SIZE, "%u\n", dev->data.pm2p5_standard_ug_m3);
    up_read(&dev->rwlock);
    return ret;
}

static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(error_code);
static DEVICE_ATTR_RO(pm2p5);

static struct attribute *pmsx003_attrs[] = {
    &dev_attr_version.attr,
    &dev_attr_error_code.attr,
    &dev_attr_pm2p5.attr,
    NULL,
};

static const struct attribute_group pmsx003_attr_group = {
    .attrs = pmsx003_attrs,
};

static int pmsx003_open(struct inode *inode, struct file *file) {
    struct pmsx003_dev *dev = container_of(inode->i_cdev, struct pmsx003_dev, cdev);
    if (atomic_inc_return(&dev->open_count) == 1) {
        dev->user_pid = current->pid;
        dev_info(dev->dev, "Device opened by PID %d\n", dev->user_pid);
    }
    file->private_data = dev;
    return 0;
}

static int pmsx003_release(struct inode *inode, struct file *file) {
    struct pmsx003_dev *dev = file->private_data;
    if (atomic_dec_and_test(&dev->open_count)) {
        dev_info(dev->dev, "Device closed by PID %d\n", dev->user_pid);
        dev->user_pid = 0;
    }
    return 0;
}

static ssize_t pmsx003_read(struct file *file, char __user *buf, size_t count, loff_t *off) {
    struct pmsx003_dev *dev = file->private_data;
    if (count < sizeof(pmsx003_data_t)) {
        dev_err(dev->dev, "pmsx003: Invalid read size\n");
        return -EINVAL;
    }

    long ret = wait_event_interruptible_timeout(dev->read_queue, dev->data.version != 0, PMSX003_READ_TIMEOUT);
    if (ret == 0) {
        dev_err(dev->dev, "pmsx003: Read timeout\n");
        return -ETIMEDOUT;
    }
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: Read interrupted: %ld\n", ret);
        return ret;
    }

    down_read(&dev->rwlock);
    smp_rmb();
    ret = copy_to_user(buf, &dev->data, sizeof(pmsx003_data_t)) ? -EFAULT : sizeof(pmsx003_data_t);
    up_read(&dev->rwlock);
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: Copy to user failed\n");
        return ret;
    }
    return ret;
}

static long pmsx003_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct pmsx003_dev *dev = file->private_data;
    long ret = 0;
    int val;
    unsigned long delay;
    pmsx003_info_t info;
    uint8_t frame[7];

    down_write(&dev->rwlock);
    down(&dev->data_sem);

    switch (cmd) {
    case PMSX003_IOCTL_SET_MODE:
        if (copy_from_user(&val, (void __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            dev_err(dev->dev, "pmsx003: IOCTL SET_MODE copy_from_user failed\n");
            break;
        }
        dev->mode = val;
        a_pmsx003_make_frame(PMSX003_COMMAND_CHANGE_WORKING_MODE, val, frame);
        pmsx003_uart_write(dev, frame, 7);
        dev_info(dev->dev, "pmsx003: Set mode to %d\n", val);
        break;
    case PMSX003_IOCTL_SET_HARD_MODE:
        if (copy_from_user(&val, (void __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            dev_err(dev->dev, "pmsx003: IOCTL SET_HARD_MODE copy_from_user failed\n");
            break;
        }
        a_pmsx003_make_frame(PMSX003_COMMAND_CHANGE_CHIP_MODE, val, frame);
        pmsx003_uart_write(dev, frame, 7);
        dev_info(dev->dev, "pmsx003: Set hard mode to %d\n", val);
        break;
    case PMSX003_IOCTL_SLEEP:
        pmsx003_sleep(dev);
        dev_info(dev->dev, "pmsx003: Entered sleep mode\n");
        break;
    case PMSX003_IOCTL_WAKE_UP:
        pmsx003_wake_up(dev);
        dev_info(dev->dev, "pmsx003: Woke up\n");
        break;
    case PMSX003_IOCTL_RESET:
        gpiod_set_value(dev->reset_gpio, 1);
        msleep(100);
        gpiod_set_value(dev->reset_gpio, 0);
        dev_info(dev->dev, "pmsx003: Reset completed\n");
        break;
    case PMSX003_IOCTL_START_PERIODIC_READ:
        if (copy_from_user(&delay, (void __user *)arg, sizeof(unsigned long))) {
            ret = -EFAULT;
            dev_err(dev->dev, "pmsx003: IOCTL START_PERIODIC_READ copy_from_user failed\n");
            break;
        }
        dev->periodic_delay = delay;
        mod_timer(&dev->periodic_timer, jiffies + msecs_to_jiffies(delay));
        queue_delayed_work(system_wq, &dev->read_work, msecs_to_jiffies(delay));
        dev_info(dev->dev, "pmsx003: Started periodic read with delay %lu ms\n", delay);
        break;
    case PMSX003_IOCTL_STOP_PERIODIC_READ:
        del_timer_sync(&dev->periodic_timer);
        cancel_delayed_work_sync(&dev->read_work);
        dev->periodic_delay = 0;
        dev_info(dev->dev, "pmsx003: Stopped periodic read\n");
        break;
    case PMSX003_IOCTL_GET_INFO:
        strncpy(info.chip_name, CHIP_NAME, sizeof(info.chip_name));
        strncpy(info.manufacturer_name, MANUFACTURER_NAME, sizeof(info.manufacturer_name));
        strncpy(info.interface, "UART", sizeof(info.interface));
        info.supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;
        info.supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;
        info.max_current_ma = MAX_CURRENT;
        info.temperature_min = TEMPERATURE_MIN;
        info.temperature_max = TEMPERATURE_MAX;
        info.driver_version = DRIVER_VERSION;
        if (copy_to_user((void __user *)arg, &info, sizeof(pmsx003_info_t))) {
            ret = -EFAULT;
            dev_err(dev->dev, "pmsx003: IOCTL GET_INFO copy_to_user failed\n");
        }
        break;
    case PMSX003_IOCTL_SET_NONBLOCK:
        if (copy_from_user(&val, (void __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            dev_err(dev->dev, "pmsx003: IOCTL SET_NONBLOCK copy_from_user failed\n");
            break;
        }
        if (val)
            file->f_flags |= O_NONBLOCK;
        else
            file->f_flags &= ~O_NONBLOCK;
        dev_info(dev->dev, "pmsx003: Set nonblock to %d\n", val);
        break;
    default:
        ret = -ENOTTY;
        dev_err(dev->dev, "pmsx003: Invalid IOCTL command\n");
    }

    up(&dev->data_sem);
    up_write(&dev->rwlock);
    return ret;
}

const struct file_operations pmsx003_fops = {
    .owner = THIS_MODULE,
    .open = pmsx003_open,
    .release = pmsx003_release,
    .read = pmsx003_read,
    .unlocked_ioctl = pmsx003_ioctl,
};

static void pmsx003_read_work_handler(struct work_struct *work) {
    struct delayed_work *dwork = to_delayed_work(work);
    struct pmsx003_dev *dev = container_of(dwork, struct pmsx003_dev, read_work);
    uint8_t buf[32];

    if (pmsx003_uart_read(dev, buf, 32) > 0) {
        down_write(&dev->rwlock);
        if (!a_pmsx003_parse_data(dev, buf, &dev->data)) {
            smp_wmb();
            complete_all(&dev->data_ready);
            wake_up_interruptible(&dev->read_queue);
            if (dev->shm_addr != NULL && dev->shm_addr != (void *)-1) {
                memcpy(dev->shm_addr, &dev->data, sizeof(pmsx003_data_t));
            }
        }
        up_write(&dev->rwlock);
    }

    if (dev->periodic_delay && !timer_pending(&dev->periodic_timer)) {
        queue_delayed_work(system_wq, &dev->read_work, msecs_to_jiffies(dev->periodic_delay));
    }
}

static int pmsx003_reader_thread(void *arg) {
    struct pmsx003_dev *dev = arg;
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 20 };
    sched_setscheduler(current, SCHED_NORMAL, &param);
    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        uint8_t buf[32];
        if (pmsx003_uart_read(dev, buf, 32) > 0) {
            down_write(&dev->rwlock);
            if (!a_pmsx003_parse_data(dev, buf, &dev->data)) {
                smp_wmb();
                complete(&dev->data_ready);
                wake_up_interruptible(&dev->read_queue);
                if (dev->shm_addr != NULL && dev->shm_addr != (void *)-1) {
                    memcpy(dev->shm_addr, &dev->data, sizeof(pmsx003_data_t));
                }
            }
            up_write(&dev->rwlock);
        }
        msleep_interruptible(100);
    }
    return 0;
}

static irqreturn_t pmsx003_irq_handler(int irq, void *data) {
    struct pmsx003_dev *dev = data;
    queue_delayed_work(system_wq, &dev->read_work, 0);
    return IRQ_HANDLED;
}

static int pmsx003_suspend(struct device *ddev) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    kthread_park(dev->reader_thread);
    del_timer_sync(&dev->periodic_timer);
    cancel_delayed_work_sync(&dev->read_work);
    dev_info(dev->dev, "pmsx003: Suspended\n");
    return 0;
}

static int pmsx003_resume(struct device *ddev) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    kthread_unpark(dev->reader_thread);
    if (dev->periodic_delay) {
        mod_timer(&dev->periodic_timer, jiffies + msecs_to_jiffies(dev->periodic_delay));
        queue_delayed_work(system_wq, &dev->read_work, msecs_to_jiffies(dev->periodic_delay));
    }
    dev_info(dev->dev, "pmsx003: Resumed\n");
    return 0;
}

static const struct dev_pm_ops pmsx003_pm_ops = {
    .suspend = pmsx003_suspend,
    .resume = pmsx003_resume,
};

static int pmsx003_sleep(struct pmsx003_dev *dev) {
    gpiod_set_value(dev->set_gpio, 0);
    kthread_park(dev->reader_thread);
    return 0;
}

static int pmsx003_wake_up(struct pmsx003_dev *dev) {
    gpiod_set_value(dev->set_gpio, 1);
    kthread_unpark(dev->reader_thread);
    return 0;
}

static int pmsx003_probe(struct platform_device *pdev) {
    struct pmsx003_dev *dev;
    struct device_node *np = pdev->dev.of_node;
    int ret;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        dev_err(&pdev->dev, "pmsx003: Failed to allocate memory\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, dev);
    dev->dev = &pdev->dev;

    init_rwsem(&dev->rwlock);
    sema_init(&dev->data_sem, 1);
    init_completion(&dev->data_ready);
    init_waitqueue_head(&dev->read_queue);
    atomic_set(&dev->open_count, 0);
    timer_setup(&dev->periodic_timer, pmsx003_timer_callback, 0);

    dev->wq = system_wq;
    INIT_DELAYED_WORK(&dev->read_work, pmsx003_read_work_handler);

    dev->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(dev->reset_gpio)) {
        ret = PTR_ERR(dev->reset_gpio);
        dev_err(&pdev->dev, "pmsx003: Failed to get reset GPIO: %d\n", ret);
        return ret;
    }

    dev->set_gpio = devm_gpiod_get_optional(&pdev->dev, "set", GPIOD_OUT_LOW);
    if (IS_ERR(dev->set_gpio)) {
        ret = PTR_ERR(dev->set_gpio);
        dev_err(&pdev->dev, "pmsx003: Failed to get set GPIO: %d\n", ret);
        return ret;
    }

    dev->uart_port = devm_ioremap(&pdev->dev, 0xFF200000, 0x1000);
    if (!dev->uart_port) {
        dev_err(&pdev->dev, "pmsx003: Failed to ioremap UART\n");
        return -ENOMEM;
    }

    ret = pmsx003_init_uart(dev);
    if (ret) {
        dev_err(&pdev->dev, "pmsx003: UART initialization failed: %d\n", ret);
        goto iounmap;
    }

    ret = alloc_chrdev_region(&dev->devno, 0, 1, "pmsx003");
    if (ret < 0) {
        dev_err(&pdev->dev, "pmsx003: Failed to allocate chrdev: %d\n", ret);
        goto deinit_uart;
    }

    cdev_init(&dev->cdev, &pmsx003_fops);
    ret = cdev_add(&dev->cdev, dev->devno, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "pmsx003: Failed to add cdev: %d\n", ret);
        goto unregister_chrdev;
    }

    dev->class = class_create(THIS_MODULE, "pmsx003_class");
    if (IS_ERR(dev->class)) {
        ret = PTR_ERR(dev->class);
        dev_err(&pdev->dev, "pmsx003: Failed to create class: %d\n", ret);
        goto del_cdev;
    }

    dev->device = device_create(dev->class, NULL, dev->devno, NULL, "pmsx003_0");
    if (IS_ERR(dev->device)) {
        ret = PTR_ERR(dev->device);
        dev_err(&pdev->dev, "pmsx003: Failed to create device: %d\n", ret);
        goto del_class;
    }

    ret = sysfs_create_group(&dev->device->kobj, &pmsx003_attr_group);
    if (ret) {
        dev_err(&pdev->dev, "pmsx003: Failed to create sysfs group: %d\n", ret);
        goto del_device;
    }

    dev->debugfs_dir = debugfs_create_dir("pmsx003", NULL);
    if (!IS_ERR(dev->debugfs_dir)) {
        debugfs_create_u8("version", 0444, dev->debugfs_dir, &dev->data.version);
        debugfs_create_u8("error_code", 0444, dev->debugfs_dir, &dev->data.error_code);
        debugfs_create_u16("pm2p5", 0444, dev->debugfs_dir, &dev->data.pm2p5_standard_ug_m3);
    } else {
        dev_warn(&pdev->dev, "pmsx003: Failed to create debugfs directory\n");
    }

    int shmid = shmget(IPC_PRIVATE, PMSX003_SHM_SIZE, IPC_CREAT | 0666);
    if (shmid < 0) {
        ret = shmid;
        dev_err(&pdev->dev, "pmsx003: Failed to create shared memory: %d\n", ret);
        goto del_sysfs;
    }
    dev->shm_addr = shmat(shmid, NULL, 0);
    if (dev->shm_addr == (void *)-1) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "pmsx003: Failed to attach shared memory\n");
        goto del_sysfs;
    }

    down_write(&dev->rwlock);
    dev->reader_thread = kthread_run(pmsx003_reader_thread, dev, "pmsx003_reader");
    up_write(&dev->rwlock);
    if (IS_ERR(dev->reader_thread)) {
        ret = PTR_ERR(dev->reader_thread);
        dev_err(&pdev->dev, "pmsx003: Failed to start reader thread: %d\n", ret);
        goto del_shm;
    }

    int irq = platform_get_irq(pdev, 0);
    if (irq > 0) {
        ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, pmsx003_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_RISING, "pmsx003_irq", dev);
        if (ret) {
            dev_err(&pdev->dev, "pmsx003: Failed to request IRQ: %d\n", ret);
            goto stop_thread;
        }
    }

    dev->inited = 1;
    dev->mode = PMSX003_MODE_PASSIVE;
    pm_runtime_enable(dev->dev);
    dev_info(&pdev->dev, "pmsx003: Driver probed successfully\n");

    return 0;

stop_thread:
    kthread_stop(dev->reader_thread);
del_shm:
    if (dev->shm_addr != (void *)-1) {
        shmdt(dev->shm_addr);
        shmctl(shmid, IPC_RMID, NULL);
    }
del_sysfs:
    sysfs_remove_group(&dev->device->kobj, &pmsx003_attr_group);
del_device:
    device_destroy(dev->class, dev->devno);
del_class:
    class_destroy(dev->class);
del_cdev:
    cdev_del(&dev->cdev);
unregister_chrdev:
    unregister_chrdev_region(&dev->devno, 1);
deinit_uart:
    pmsx003_deinit_uart(dev);
iounmap:
    iounmap(dev->uart_port);
    return ret;
}

static int pmsx003_remove(struct platform_device *pdev) {
    struct pmsx003_dev *dev = platform_get_drvdata(pdev);

    pm_runtime_disable(dev->dev);
    del_timer_sync(&dev->periodic_timer);
    cancel_delayed_work_sync(&dev->read_work);
    kthread_stop(dev->reader_thread);
    debugfs_remove_recursive(dev->debugfs_dir);
    sysfs_remove_group(&dev->device->kobj, &pmsx003_attr_group);
    pmsx003_deinit_uart(dev);
    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(&dev->devno, 1);
    iounmap(dev->uart_port);
    if (dev->shm_addr != (void *)-1) {
        shmdt(dev->shm_addr);
        shmctl(shmget(IPC_PRIVATE, 0, 0), IPC_RMID, NULL);
    }

    dev_info(&pdev->dev, "pmsx003: Driver removed\n");
    return 0;
}

static void pmsx003_shutdown(struct platform_device *pdev) {
    struct pmsx003_dev *dev = platform_get_drvdata(pdev);
    del_timer_sync(&dev->periodic_timer);
    cancel_delayed_work_sync(&dev->read_work);
    pmsx003_sleep(dev);
    dev_info(&pdev->dev, "pmsx003: Driver shutdown\n");
}

static const struct of_device_id pmsx003_of_match[] = {
    { .compatible = "plantower,pmsx003" },
    { }
};
MODULE_DEVICE_TABLE(of, pmsx003_of_match);

static struct platform_driver pmsx003_driver = {
    .probe = pmsx003_probe,
    .remove = pmsx003_remove,
    .shutdown = pmsx003_shutdown,
    .driver = {
        .name = "pmsx003",
        .of_match_table = pmsx003_of_match,
        .pm = &pmsx003_pm_ops,
    },
};

module_platform_driver(pmsx003_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Advanced PMSX003 Kernel Driver with Full Concurrency");
MODULE_VERSION("1.0.2");