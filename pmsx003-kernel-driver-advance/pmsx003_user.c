#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include "pmsx003.h"

static volatile sig_atomic_t keep_running = 1;

static void signal_handler(int sig) {
    keep_running = 0;
}

int main(int argc, char *argv[]) {
    int fd;
    pmsx003_data_t data;
    pmsx003_info_t info;
    int nonblock = 0;

    if (argc > 1 && strcmp(argv[1], "--nonblock") == 0)
        nonblock = 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    fd = open("/dev/pmsx003_0", O_RDWR | (nonblock ? O_NONBLOCK : 0));
    if (fd < 0) {
        perror("open failed");
        return 1;
    }

    if (nonblock) {
        if (ioctl(fd, PMSX003_IOCTL_SET_NONBLOCK, &nonblock) < 0) {
            perror("ioctl set nonblock failed");
            close(fd);
            return 1;
        }
    }

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

    void *shm_addr = shmat(shmget(IPC_PRIVATE, 4096, IPC_CREAT | 0666), NULL, 0);
    if (shm_addr == (void *)-1) {
        perror("shared memory attach failed");
    }

    while (keep_running) {
        if (read(fd, &data, sizeof(data)) != sizeof(data)) {
            if (errno == EAGAIN && nonblock)
                continue;
            perror("read failed");
            break;
        }
        if (data.error_code != 0) {
            fprintf(stderr, "Data error code: 0x%02X\n", data.error_code);
            continue;
        }
        printf("PM2.5: %d ug/m3\n", data.pm2p5_standard_ug_m3);
        if (shm_addr) {
            pmsx003_data_t *shm_data = (pmsx003_data_t *)shm_addr;
            printf("Shared PM2.5: %d ug/m3\n", shm_data->pm2p5_standard_ug_m3);
        }
        sleep(1);
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

    if (shm_addr != (void *)-1)
        shmdt(shm_addr);

    close(fd);
    return 0;
}