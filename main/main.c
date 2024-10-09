#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <math.h> 

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct Mouse {
    int axis;
    int val;
} mouse;

QueueHandle_t xQueueAdc;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    mouse mouse;
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    // Define sample period
    #define SAMPLE_PERIOD (0.01f) // 10 ms

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Convert raw data to physical units
        FusionVector gyroscope;
        gyroscope.axis.x = gyro[0] / 131.0f;   // Convert to degrees/s
        gyroscope.axis.y = gyro[1] / 131.0f;
        gyroscope.axis.z = gyro[2] / 131.0f;

        FusionVector accelerometer;
        accelerometer.axis.x = acceleration[0] / 16384.0f;  // Convert to g
        accelerometer.axis.y = acceleration[1] / 16384.0f;
        accelerometer.axis.z = acceleration[2] / 16384.0f;

        // Update FusionAhrs without magnetometer data
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Get Euler angles from the quaternion
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        if(fabs(euler.angle.roll) >= 10){
            mouse.axis = 1;
            mouse.val = -euler.angle.roll;
            xQueueSend(xQueueAdc, &mouse, portMAX_DELAY);
        }
        if(fabs(euler.angle.yaw) >= 10){
            mouse.axis = 0;
            mouse.val = -euler.angle.yaw;
            xQueueSend(xQueueAdc, &mouse, portMAX_DELAY);
        }
        if(fabs(accelerometer.axis.y) >= 1.5){
            mouse.axis = 2;
            mouse.val = 1;
            xQueueSend(xQueueAdc, &mouse, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }


}

void uart_task(void *p) {
    mouse mouse;

    while (1) {
        if(xQueueReceive(xQueueAdc, &mouse, portMAX_DELAY)){
            uart_putc(uart0, mouse.axis);
            uart_putc(uart0, mouse.val >> 8);
            uart_putc(uart0, mouse.val & 0xFF);
            uart_putc(uart0, -1);
        }
    }
}

int main() {
    stdio_init_all();

    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    xQueueAdc = xQueueCreate(32, sizeof(mouse));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
