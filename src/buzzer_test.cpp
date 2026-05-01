/**
 * @file buzzer_test2.cpp
 * @brief QCar2 buzzer finder v2: PWM frequency mode + Jetson GPIO toggle.
 *
 * Build:
 *   g++ -o buzzer_test2 buzzer_test2.cpp -lhil -lquanser_devices -lquanser_runtime -lquanser_common -lm
 * Run:
 *   ./buzzer_test2
 */

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

#include "quanser/hil.h"
#include "quanser/quanser_messages.h"

static void print_error(const char* label, t_error result)
{
    char msg[512];
    msg_get_error_messageA(NULL, result, msg, sizeof(msg));
    printf("  [FAIL] %s: %s\n", label, msg);
}

// Software bit-bang a GPIO to produce a tone
static void gpio_tone(int gpio_num, int freq_hz, int duration_ms)
{
    char path[128];

    // Export GPIO
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) {
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
        write(fd, buf, len);
        close(fd);
    }
    usleep(100000); // wait for export

    // Set direction to output
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        printf("  [FAIL] Cannot open GPIO %d direction\n", gpio_num);
        return;
    }
    write(fd, "out", 3);
    close(fd);

    // Open value file
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        printf("  [FAIL] Cannot open GPIO %d value\n", gpio_num);
        return;
    }

    // Toggle at frequency for duration
    int half_period_us = 500000 / freq_hz;
    int cycles = freq_hz * duration_ms / 1000;

    printf("[toggling %d cycles]... ", cycles);
    for (int i = 0; i < cycles; ++i) {
        write(fd, "1", 1);
        lseek(fd, 0, SEEK_SET);
        usleep(half_period_us);
        write(fd, "0", 1);
        lseek(fd, 0, SEEK_SET);
        usleep(half_period_us);
    }
    close(fd);

    // Unexport
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd >= 0) {
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
        write(fd, buf, len);
        close(fd);
    }
}

int main()
{
    t_card card;
    t_error result;

    printf("=== QCar2 Buzzer Channel Finder v2 ===\n");
    printf("Listen carefully after each test!\n\n");

    // Open HIL
    result = hil_open("qcar2", "0", &card);
    if (result < 0) {
        print_error("hil_open", result);
        return 1;
    }
    printf("[OK] HIL card opened\n\n");

    // ============================================================
    // TEST 1: PWM in FREQUENCY mode (passive buzzer needs AC signal)
    // ============================================================
    printf("=== TEST 1: PWM frequency mode ===\n");
    for (t_uint32 ch = 0; ch <= 1; ++ch) {
        t_uint32 channels[] = {ch};

        // Set to frequency mode
        t_pwm_mode modes[] = {PWM_FREQUENCY_MODE};
        result = hil_set_pwm_mode(card, channels, 1, modes);
        if (result < 0) {
            printf("  PWM ch %u set_mode: ", ch);
            print_error("hil_set_pwm_mode", result);
            continue;
        }

        // Write 1 kHz frequency
        t_double freq = 1000.0;
        printf("  PWM ch %u freq=1000Hz... ", ch);
        result = hil_write_pwm(card, channels, 1, &freq);
        if (result >= 0) {
            printf("[OK] Waiting 2s...\n");
            sleep(2);
            freq = 0.0;
            hil_write_pwm(card, channels, 1, &freq);
        } else {
            print_error("hil_write_pwm", result);
        }

        // Reset to duty cycle mode
        modes[0] = PWM_DUTY_CYCLE_MODE;
        hil_set_pwm_mode(card, channels, 1, modes);
    }

    // ============================================================
    // TEST 2: PWM duty cycle with set_pwm_frequency (set the carrier freq)
    // ============================================================
    printf("\n=== TEST 2: PWM duty + carrier freq ===\n");
    for (t_uint32 ch = 0; ch <= 1; ++ch) {
        t_uint32 channels[] = {ch};

        // Set carrier frequency to 1 kHz
        t_double carrier = 1000.0;
        result = hil_set_pwm_frequency(card, channels, 1, &carrier);
        if (result < 0) {
            printf("  PWM ch %u set_freq: ", ch);
            print_error("hil_set_pwm_frequency", result);
            continue;
        }

        // Write 50% duty cycle
        t_double duty = 0.5;
        printf("  PWM ch %u carrier=1kHz duty=50%%... ", ch);
        result = hil_write_pwm(card, channels, 1, &duty);
        if (result >= 0) {
            printf("[OK] Waiting 2s...\n");
            sleep(2);
            duty = 0.0;
            hil_write_pwm(card, channels, 1, &duty);
        } else {
            print_error("hil_write_pwm", result);
        }
    }

    // ============================================================
    // TEST 3: Digital output rapid toggle (software PWM)
    // ============================================================
    printf("\n=== TEST 3: Digital output rapid toggle (1kHz) ===\n");
    printf("  Testing channels 0-10 with fast toggle...\n");
    for (t_uint32 ch = 0; ch <= 10; ++ch) {
        t_uint32 channels[] = {ch};
        t_boolean on = 1, off = 0;

        printf("  Digital ch %u toggle 1kHz for 1s... ", ch);
        // Toggle at ~1kHz for 1 second
        for (int i = 0; i < 1000; ++i) {
            hil_write_digital(card, channels, 1, &on);
            usleep(500);  // 500us
            hil_write_digital(card, channels, 1, &off);
            usleep(500);  // 500us = 1ms period = 1kHz
        }
        printf("done\n");
    }

    hil_close(card);

    // ============================================================
    // TEST 4: Jetson GPIO toggle (bypass HIL entirely)
    // ============================================================
    printf("\n=== TEST 4: Jetson GPIO direct toggle ===\n");
    // Common Jetson Orin GPIO pins that might be routed to the QCar2 buzzer
    // These are the sysfs GPIO numbers for Jetson AGX Orin
    int gpio_candidates[] = {
        348, 349, 350, 351,   // GPIO group near expansion header
        316, 317, 318, 319,   // more GPIOs
        393, 394, 395, 396,   // PWM-capable pins
        -1
    };

    for (int i = 0; gpio_candidates[i] != -1; ++i) {
        int g = gpio_candidates[i];
        printf("  GPIO %d 1kHz for 1s... ", g);
        gpio_tone(g, 1000, 1000);
        printf("done\n");
    }

    printf("\n=== All tests done. Did you hear anything? ===\n");
    printf("Note: if digital toggle test 3 produced sound on a channel,\n");
    printf("that's the buzzer channel via HIL digital output.\n");
    return 0;
}
