#include "pmsx003.h"

#define PMSX003_UART_BAUD 9600
#define PMSX003_UART_BITS 8
#define PMSX003_UART_PARITY 'N'
#define PMSX003_UART_STOP 1
#define PMSX003_RETRY_COUNT 3
#define PMSX003_UART_PORTS 1

static struct uart_driver pmsx003_uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = "pmsx003_uart",
    .dev_name = "ttyPMS",
    .nr = PMSX003_UART_PORTS,
};

static const struct uart_ops pmsx003_uart_ops = {
    /* Implement all required uart_ops: tx_ready, set_mctrl, etc. */
    /* For brevity, assume standard impl; in real, fill all */
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
        dev_err(dev->dev, "pmsx003: lrc check error.\n");
        return 1;
    }
    if (input[0] != 0x42 || input[1] != 0x4D) return 1;

    uint16_t len = ((uint16_t)input[2] << 8) | input[3];
    if (len != 4) return 1;
    if (input[4] != command) return 1;
    *data = input[5];

    return 0;
}

static uint8_t a_pmsx003_parse_data(struct pmsx003_dev *dev, uint8_t input[32], pmsx003_data_t *data) {
    uint8_t i;
    uint16_t lrc = 0, lrc_check;

    for (i = 0; i < 30; i++) lrc += input[i];
    lrc_check = ((uint16_t)input[30] << 8) | input[31];
    if (lrc != lrc_check) {
        dev_err(dev->dev, "pmsx003: lrc check error.\n");
        return 1;
    }
    if (input[0] != 0x42 || input[1] != 0x4D) return 1;

    uint16_t len = ((uint16_t)input[2] << 8) | input[3];
    if (len != 28) return 1;

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
        dev_err(dev->dev, "pmsx003: uart write failed.\n");
        return 1;
    }
    return 0;
}

static uint16_t pmsx003_uart_read(struct pmsx003_dev *dev, uint8_t *buf, uint16_t len) {
    int ret = uart_read(dev->uart_port, buf, len);
    if (ret < 0) {
        dev_err(dev->dev, "pmsx003: uart read failed.\n");
        return 0;
    }
    return (uint16_t)ret;
}

static uint8_t pmsx003_uart_flush(struct pmsx003_dev *dev) {
    uart_flush_tx(dev->uart_port);
    return 0;
}

static int pmsx003_init_uart(struct pmsx003_dev *dev) {
    int ret;

    ret = uart_register_driver(&pmsx003_uart_driver);
    if (ret) return ret;

    dev->uart_port = uart_add_one_port(&pmsx003_uart_driver, dev->uart_port);
    if (ret) goto unreg;

    uart_set_options(dev->uart_port, PMSX003_UART_BAUD, PMSX003_UART_BITS, PMSX003_UART_PARITY, PMSX003_UART_STOP);

    return 0;

unreg:
    uart_unregister_driver(&pmsx003_uart_driver);
    return ret;
}

static void pmsx003_deinit_uart(struct pmsx003_dev *dev) {
    uart_remove_one_port(&pmsx003_uart_driver, dev->uart_port);
    uart_unregister_driver(&pmsx003_uart_driver);
}

static uint8_t pmsx003_read_data(struct pmsx003_dev *dev, pmsx003_data_t *data) {
    uint8_t output[7];
    uint8_t input[32];
    uint16_t len;
    uint8_t res;
    int retry = PMSX003_RETRY_COUNT;

    while (retry--) {
        if (dev->mode == PMSX003_MODE_ACTIVE) {
            len = pmsx003_uart_read(dev, input, 32);
            if (len != 32) {
                dev_warn(dev->dev, "Active read len error, retry %d\n", retry);
                continue;
            }
            res = pmsx003_uart_flush(dev);
            if (res != 0) continue;
            res = a_pmsx003_parse_data(dev, input, data);
            if (res != 0) continue;
            if (data->error_code != 0) continue;
            return 0;
        } else {
            a_pmsx003_make_frame(PMSX003_COMMAND_READ, 0x00, output);
            res = pmsx003_uart_flush(dev);
            if (res != 0) continue;
            res = pmsx003_uart_write(dev, output, 7);
            if (res != 0) continue;
            msleep(100);
            len = pmsx003_uart_read(dev, input, 32);
            if (len != 32) continue;
            res = a_pmsx003_parse_data(dev, input, data);
            if (res != 0) continue;
            if (data->error_code != 0) continue;
            return 0;
        }
    }
    dev_err(dev->dev, "Read data failed after retries.\n");
    return 1;
}

static uint8_t pmsx003_set_mode(struct pmsx003_dev *dev, pmsx003_mode_t mode) {
    uint8_t output[7];
    uint8_t input[8];
    uint8_t data;
    uint16_t len;
    uint8_t res;
    int retry = PMSX003_RETRY_COUNT;

    while (retry--) {
        a_pmsx003_make_frame(PMSX003_COMMAND_CHANGE_WORKING_MODE, (mode == PMSX003_MODE_ACTIVE ? 0x01 : 0x00), output);
        res = pmsx003_uart_flush(dev);
        if (res != 0) continue;
        res = pmsx003_uart_write(dev, output, 7);
        if (res != 0) continue;
        msleep(100);
        len = pmsx003_uart_read(dev, input, 8);
        if (len != 8) continue;
        res = a_pmsx003_parse_frame(dev, input, PMSX003_COMMAND_CHANGE_WORKING_MODE, &data);
        if (res != 0) continue;
        return 0;
    }
    dev_err(dev->dev, "Set mode failed after retries.\n");
    return 1;
}

static uint8_t pmsx003_sleep(struct pmsx003_dev *dev) {
    uint8_t output[7];
    uint8_t input[8];
    uint8_t data;
    uint16_t len;
    uint8_t res;
    int retry = PMSX003_RETRY_COUNT;

    while (retry--) {
        a_pmsx003_make_frame(PMSX003_COMMAND_CHANGE_CHIP_MODE, 0x00, output);
        res = pmsx003_uart_flush(dev);
        if (res != 0) continue;
        res = pmsx003_uart_write(dev, output, 7);
        if (res != 0) continue;
        msleep(100);
        len = pmsx003_uart_read(dev, input, 8);
        if (len != 8) continue;
        res = a_pmsx003_parse_frame(dev, input, PMSX003_COMMAND_CHANGE_CHIP_MODE, &data);
        if (res != 0) continue;
        return 0;
    }
    dev_err(dev->dev, "Sleep failed after retries.\n");
    return 1;
}

static uint8_t pmsx003_wake_up(struct pmsx003_dev *dev) {
    uint8_t output[7];
    uint8_t input[8];
    uint8_t data;
    uint16_t len;
    uint8_t res;
    int retry = PMSX003_RETRY_COUNT;

    while (retry--) {
        a_pmsx003_make_frame(PMSX003_COMMAND_CHANGE_CHIP_MODE, 0x01, output);
        res = pmsx003_uart_flush(dev);
        if (res != 0) continue;
        res = pmsx003_uart_write(dev, output, 7);
        if (res != 0) continue;
        msleep(100);
        len = pmsx003_uart_read(dev, input, 8);
        if (len != 8) continue;
        res = a_pmsx003_parse_frame(dev, input, PMSX003_COMMAND_CHANGE_CHIP_MODE, &data);
        if (res != 0) continue;
        return 0;
    }
    dev_err(dev->dev, "Wake up failed after retries.\n");
    return 1;
}

static uint8_t pmsx003_reset(struct pmsx003_dev *dev) {
    gpiod_set_value(dev->reset_gpio, 0);
    msleep(100);
    gpiod_set_value(dev->reset_gpio, 1);
    msleep(1000);  /* Wait for stabilization */
    return 0;
}

static uint8_t pmsx003_set_hard_mode(struct pmsx003_dev *dev, pmsx003_hard_mode_t mode) {
    gpiod_set_value(dev->set_gpio, (mode == PMSX003_HARD_MODE_NORMAL ? 1 : 0));
    msleep(100);
    return 0;
}

static void pmsx003_read_work(struct work_struct *work) {
    struct pmsx003_dev *dev = container_of(to_delayed_work(work), struct pmsx003_dev, read_work);
    mutex_lock(&dev->lock);
    if (pmsx003_read_data(dev, &dev->data)) {
        dev_err(dev->dev, "Periodic read failed");
    }
    mutex_unlock(&dev->lock);

    if (dev->periodic_delay && !cancel_delayed_work(&dev->read_work)) {
        queue_delayed_work(dev->wq, &dev->read_work, msecs_to_jiffies(dev->periodic_delay));
    }
}

static int pmsx003_open(struct inode *inode, struct file *file) {
    struct pmsx003_dev *dev = container_of(inode->i_cdev, struct pmsx003_dev, cdev);
    if (!try_module_get(THIS_MODULE)) return -ENODEV;
    file->private_data = dev;
    return 0;
}

static int pmsx003_release(struct inode *inode, struct file *file) {
    module_put(THIS_MODULE);
    return 0;
}

static ssize_t pmsx003_read(struct file *file, char __user *buf, size_t count, loff_t *off) {
    struct pmsx003_dev *dev = file->private_data;
    size_t to_copy = min(count, sizeof(pmsx003_data_t));

    mutex_lock(&dev->lock);
    if (copy_to_user(buf, &dev->data, to_copy)) {
        mutex_unlock(&dev->lock);
        return -EFAULT;
    }
    mutex_unlock(&dev->lock);

    return to_copy;
}

static ssize_t pmsx003_write(struct file *file, const char __user *buf, size_t count, loff_t *off) {
    struct pmsx003_dev *dev = file->private_data;
    uint8_t *kbuf = kzalloc(count, GFP_KERNEL);
    if (!kbuf) return -ENOMEM;

    if (copy_from_user(kbuf, buf, count)) {
        kfree(kbuf);
        return -EFAULT;
    }

    mutex_lock(&dev->lock);
    if (pmsx003_uart_write(dev, kbuf, count)) {
        mutex_unlock(&dev->lock);
        kfree(kbuf);
        return -EIO;
    }
    mutex_unlock(&dev->lock);

    kfree(kbuf);
    return count;
}

static long pmsx003_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    return pmsx003_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}

static long pmsx003_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct pmsx003_dev *dev = file->private_data;
    int mode;
    unsigned long delay;
    pmsx003_info_t info;

    if (_IOC_TYPE(cmd) != PMSX003_MAGIC) return -ENOTTY;

    mutex_lock(&dev->lock);
    switch (cmd) {
        case PMSX003_IOCTL_SET_MODE:
            if (get_user(mode, (int __user *)arg)) {
                mutex_unlock(&dev->lock);
                return -EFAULT;
            }
            if (mode != PMSX003_MODE_PASSIVE && mode != PMSX003_MODE_ACTIVE) {
                mutex_unlock(&dev->lock);
                return -EINVAL;
            }
            if (pmsx003_set_mode(dev, mode)) {
                mutex_unlock(&dev->lock);
                return -EIO;
            }
            dev->mode = mode;
            break;
        case PMSX003_IOCTL_SET_HARD_MODE:
            if (get_user(mode, (int __user *)arg)) {
                mutex_unlock(&dev->lock);
                return -EFAULT;
            }
            if (mode != PMSX003_HARD_MODE_SLEEP && mode != PMSX003_HARD_MODE_NORMAL) {
                mutex_unlock(&dev->lock);
                return -EINVAL;
            }
            if (pmsx003_set_hard_mode(dev, mode)) {
                mutex_unlock(&dev->lock);
                return -EIO;
            }
            break;
        case PMSX003_IOCTL_SLEEP:
            if (pmsx003_sleep(dev)) {
                mutex_unlock(&dev->lock);
                return -EIO;
            }
            break;
        case PMSX003_IOCTL_WAKE_UP:
            if (pmsx003_wake_up(dev)) {
                mutex_unlock(&dev->lock);
                return -EIO;
            }
            break;
        case PMSX003_IOCTL_RESET:
            if (pmsx003_reset(dev)) {
                mutex_unlock(&dev->lock);
                return -EIO;
            }
            break;
        case PMSX003_IOCTL_START_PERIODIC_READ:
            if (get_user(delay, (unsigned long __user *)arg)) {
                mutex_unlock(&dev->lock);
                return -EFAULT;
            }
            if (delay < 100) {  /* Min delay */
                mutex_unlock(&dev->lock);
                return -EINVAL;
            }
            dev->periodic_delay = delay;
            queue_delayed_work(dev->wq, &dev->read_work, msecs_to_jiffies(delay));
            break;
        case PMSX003_IOCTL_STOP_PERIODIC_READ:
            dev->periodic_delay = 0;
            cancel_delayed_work_sync(&dev->read_work);
            break;
        case PMSX003_IOCTL_GET_INFO:
            strncpy(info.chip_name, CHIP_NAME, 32);
            strncpy(info.manufacturer_name, MANUFACTURER_NAME, 32);
            strncpy(info.interface, "UART", 8);
            info.supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;
            info.supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;
            info.max_current_ma = MAX_CURRENT;
            info.temperature_min = TEMPERATURE_MIN;
            info.temperature_max = TEMPERATURE_MAX;
            info.driver_version = DRIVER_VERSION;
            if (copy_to_user((void __user *)arg, &info, sizeof(pmsx003_info_t))) {
                mutex_unlock(&dev->lock);
                return -EFAULT;
            }
            break;
        default:
            mutex_unlock(&dev->lock);
            return -ENOTTY;
    }
    mutex_unlock(&dev->lock);
    return 0;
}

static const struct file_operations pmsx003_fops = {
    .owner = THIS_MODULE,
    .open = pmsx003_open,
    .release = pmsx003_release,
    .read = pmsx003_read,
    .write = pmsx003_write,
    .unlocked_ioctl = pmsx003_ioctl,
    .compat_ioctl = pmsx003_compat_ioctl,
};

static ssize_t pmsx003_pm25_show(struct device *ddev, struct device_attribute *attr, char *buf) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    mutex_lock(&dev->lock);
    ssize_t len = scnprintf(buf, PAGE_SIZE, "%u\n", dev->data.pm2p5_standard_ug_m3);
    mutex_unlock(&dev->lock);
    return len;
}

static DEVICE_ATTR_RO(pmsx003_pm25);

static struct attribute *pmsx003_attrs[] = {
    &dev_attr_pmsx003_pm25.attr,
    NULL
};

static const struct attribute_group pmsx003_attr_group = {
    .attrs = pmsx003_attrs,
};

static int pmsx003_suspend(struct device *ddev) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    cancel_delayed_work_sync(&dev->read_work);
    pmsx003_sleep(dev);  /* Put sensor to sleep */
    return 0;
}

static int pmsx003_resume(struct device *ddev) {
    struct pmsx003_dev *dev = dev_get_drvdata(ddev);
    pmsx003_wake_up(dev);
    if (dev->periodic_delay) queue_delayed_work(dev->wq, &dev->read_work, msecs_to_jiffies(dev->periodic_delay));
    return 0;
}

static const struct dev_pm_ops pmsx003_pm_ops = {
    .suspend = pmsx003_suspend,
    .resume = pmsx003_resume,
};

static int pmsx003_probe(struct platform_device *pdev) {
    struct pmsx003_dev *dev;
    int ret;
    struct device_node *np = pdev->dev.of_node;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev) return -ENOMEM;

    dev->dev = &pdev->dev;
    platform_set_drvdata(pdev, dev);

    mutex_init(&dev->lock);
    dev->wq = alloc_ordered_workqueue("pmsx003_wq", 0);
    if (!dev->wq) return -ENOMEM;
    INIT_DELAYED_WORK(&dev->read_work, pmsx003_read_work);

    dev->reset_gpio = devm_gpiod_get(dev->dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(dev->reset_gpio)) return PTR_ERR(dev->reset_gpio);

    dev->set_gpio = devm_gpiod_get(dev->dev, "set", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(dev->set_gpio)) return PTR_ERR(dev->set_gpio);

    /* Get UART from DT */
    dev->uart_port = of_iomap(np, 0);
    if (!dev->uart_port) return -ENOMEM;

    ret = pmsx003_init_uart(dev);
    if (ret) goto iounmap;

    ret = alloc_chrdev_region(&dev->devno, 0, 1, "pmsx003");
    if (ret < 0) return ret;

    cdev_init(&dev->cdev, &pmsx003_fops);
    ret = cdev_add(&dev->cdev, dev->devno, 1);
    if (ret < 0) goto unregister_chrdev;

    dev->class = class_create(THIS_MODULE, "pmsx003_class");
    if (IS_ERR_OR_NULL(dev->class)) {
        ret = PTR_ERR(dev->class);
        goto del_cdev;
    }

    dev->device = device_create(dev->class, NULL, dev->devno, NULL, "pmsx003_%d", MINOR(dev->devno));
    if (IS_ERR_OR_NULL(dev->device)) {
        ret = PTR_ERR(dev->device);
        goto del_class;
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &pmsx003_attr_group);
    if (ret) goto del_device;

    dev->debugfs_dir = debugfs_create_dir("pmsx003", NULL);
    if (dev->debugfs_dir) {
        debugfs_create_u32("version", 0444, dev->debugfs_dir, &dev->data.version);
        debugfs_create_u8("error_code", 0444, dev->debugfs_dir, &dev->data.error_code);
        /* Add more for full debug */
    }

    dev->inited = 1;
    dev->mode = PMSX003_MODE_PASSIVE;
    pm_runtime_enable(dev->dev);

    return 0;

del_device:
    device_destroy(dev->class, dev->devno);
del_class:
    class_destroy(dev->class);
del_cdev:
    cdev_del(&dev->cdev);
unregister_chrdev:
    unregister_chrdev_region(dev->devno, 1);
iounmap:
    iounmap(dev->uart_port);
    return ret;
}

static int pmsx003_remove(struct platform_device *pdev) {
    struct pmsx003_dev *dev = platform_get_drvdata(pdev);

    pm_runtime_disable(dev->dev);
    debugfs_remove_recursive(dev->debugfs_dir);
    sysfs_remove_group(&pdev->dev.kobj, &pmsx003_attr_group);
    destroy_workqueue(dev->wq);
    pmsx003_deinit_uart(dev);
    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, 1);
    iounmap(dev->uart_port);

    return 0;
}

static void pmsx003_shutdown(struct platform_device *pdev) {
    struct pmsx003_dev *dev = platform_get_drvdata(pdev);
    cancel_delayed_work_sync(&dev->read_work);
    pmsx003_sleep(dev);
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

MODULE_LICENSE("MIT");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("PMSx003 Kernel Driver");
MODULE_VERSION("1.0.0");