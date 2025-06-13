/*
 * LED blink with FreeRTOS + MPU6050
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include "mpu6050.h"  // <- Incluir o driver

struct led_task_arg {
    int gpio;
    int delay;
};

void led_task(void *p) {
    struct led_task_arg *a = (struct led_task_arg *)p;

    gpio_init(a->gpio);
    gpio_set_dir(a->gpio, GPIO_OUT);
    while (true) {
        gpio_put(a->gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(a->delay));
        gpio_put(a->gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(a->delay));
    }
}

// Nova task para o MPU6050
void mpu6050_task(void *p) {
    // Configurar o MPU6050
    imu_c mpu_config;
    mpu6050_set_config(&mpu_config, i2c0, 4, 5, MPU6050_ACCEL_SCALE_2G);
    
    // Tentar inicializar o MPU6050
    if (!mpu6050_init(mpu_config)) {
        printf("Erro ao inicializar MPU6050!\n");
        vTaskDelete(NULL); // Terminar a task se falhar
        return;
    }
    
    printf("MPU6050 inicializado com sucesso!\n");
    
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
    
    while (true) {
        // Ler dados do acelerômetro
        if (mpu6050_read_acc(mpu_config, accel)) {
            printf("Accel - X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
        }
        
        // Ler dados do giroscópio
        if (mpu6050_read_gyro(mpu_config, gyro)) {
            printf("Gyro - X: %d, Y: %d, Z: %d\n", gyro[0], gyro[1], gyro[2]);
        }
        
        // Ler temperatura
        if (mpu6050_read_temp(mpu_config, &temp)) {
            float temp_celsius = (temp / 340.0) + 36.53;
            printf("Temp: %.2f°C\n", temp_celsius);
        }
        
        printf("---\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Ler a cada 1 segundo
    }
}

int main() {
    stdio_init_all();
    printf("Start LED blink + MPU6050\n");

    // Tasks dos LEDs (suas tasks originais)
    struct led_task_arg arg1 = {20, 100};
    xTaskCreate(led_task, "LED_Task 1", 256, &arg1, 1, NULL);

    struct led_task_arg arg2 = {21, 200};
    xTaskCreate(led_task, "LED_Task 2", 256, &arg2, 1, NULL);

    struct led_task_arg arg3 = {22, 300};
    xTaskCreate(led_task, "LED_Task 3", 256, &arg3, 1, NULL);

    // Nova task do MPU6050
    xTaskCreate(mpu6050_task, "MPU6050_Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}