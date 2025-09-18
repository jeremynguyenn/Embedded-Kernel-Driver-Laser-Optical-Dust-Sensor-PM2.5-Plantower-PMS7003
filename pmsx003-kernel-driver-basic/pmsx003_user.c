#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include "pmsx003.h"

int main() {
    int fd = open("/dev/pmsx003_0", O_RDWR);  /* Adjust for minor number */
    if (fd < 0) {
        perror("open failed");
        return 1;
    }

    pmsx003_data_t data;
    int mode = PMSX003_MODE_ACTIVE;
    if (ioctl(fd, PMSX003_IOCTL_SET_MODE, &mode) < 0) {
        perror("ioctl set mode failed");
        close(fd);
        return 1;
    }

    unsigned long delay = 1000;
    if (ioctl(fd, PMSX003_IOCTL_START_PERIODIC_READ, &delay) < 0) {
        perror("ioctl start periodic failed");
        close(fd);
        return 1;
    }

    /* Loop for periodic read test with error handling */
    for (int i = 0; i < 10; i++) {  /* Read 10 times */
        if (read(fd, &data, sizeof(data)) != sizeof(data)) {
            perror("read failed");
            break;
        }
        if (data.error_code != 0) {
            fprintf(stderr, "Data error code: 0x%02X\n", data.error_code);
        }
        printf("PM2.5: %d ug/m3 (iteration %d)\n", data.pm2p5_standard_ug_m3, i);
        sleep(1);  /* Wait for next data */
    }

    if (ioctl(fd, PMSX003_IOCTL_STOP_PERIODIC_READ) < 0) {
        perror("ioctl stop periodic failed");
    }

    mode = PMSX003_HARD_MODE_NORMAL;
    if (ioctl(fd, PMSX003_IOCTL_SET_HARD_MODE, &mode) < 0) {
        perror("ioctl set hard mode failed");
    }

    if (ioctl(fd, PMSX003_IOCTL_SLEEP) < 0) {
        perror("ioctl sleep failed");
    }

    if (ioctl(fd, PMSX003_IOCTL_WAKE_UP) < 0) {
        perror("ioctl wake up failed");
    }

    if (ioctl(fd, PMSX003_IOCTL_RESET) < 0) {
        perror("ioctl reset failed");
    }

    pmsx003_info_t info;
    if (ioctl(fd, PMSX003_IOCTL_GET_INFO, &info) < 0) {
        perror("ioctl get info failed");
    } else {
        printf("Chip name: %s\n", info.chip_name);
        printf("Manufacturer: %s\n", info.manufacturer_name);
        printf("Interface: %s\n", info.interface);
        printf("Supply voltage min: %.1fV\n", info.supply_voltage_min_v);
        printf("Supply voltage max: %.1fV\n", info.supply_voltage_max_v);
        printf("Max current: %.1fmA\n", info.max_current_ma);
        printf("Temperature min: %.1fC\n", info.temperature_min);
        printf("Temperature max: %.1fC\n", info.temperature_max);
        printf("Driver version: %u\n", info.driver_version);
    }

    close(fd);
    return 0;
}