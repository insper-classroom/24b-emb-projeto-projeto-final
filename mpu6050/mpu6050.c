#include "mpu6050.h"

// Função auxiliar para escrever um byte em um registrador
static int mpu6050_write_reg(imu_c config, uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    int result = i2c_write_blocking(config.i2c, MPU6050_ADDR, buffer, 2, false);
    return (result == 2) ? 1 : 0;
}

// Função auxiliar para ler dados de um registrador
static int mpu6050_read_reg(imu_c config, uint8_t reg, uint8_t *data, size_t len) {
    int result = i2c_write_blocking(config.i2c, MPU6050_ADDR, &reg, 1, true);
    if (result != 1) return 0;
    
    result = i2c_read_blocking(config.i2c, MPU6050_ADDR, data, len, false);
    return (result == len) ? 1 : 0;
}

void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

int mpu6050_init(imu_c config) {
    // Inicializar I2C
    i2c_init(config.i2c, 400 * 1000); // 400kHz
    
    // Configurar pinos I2C
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);
    
    // Verificar se o dispositivo está presente
    uint8_t who_am_i;
    if (!mpu6050_read_reg(config, MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        return 0;
    }
    
    // Sair do modo sleep
    if (!mpu6050_write_reg(config, MPU6050_REG_PWR_MGMT_1, 0x00)) {
        return 0;
    }
    
    // Configurar escala do acelerômetro
    uint8_t accel_config = (config.acc_scale << 3);
    if (!mpu6050_write_reg(config, MPU6050_REG_ACCEL_CONFIG, accel_config)) {
        return 0;
    }
    
    sleep_ms(100); // Aguardar estabilização
    
    return 1;
}

int mpu6050_reset(imu_c config) {
    // Reset do dispositivo
    if (!mpu6050_write_reg(config, MPU6050_REG_PWR_MGMT_1, 0x80)) {
        return 0;
    }
    
    sleep_ms(100); // Aguardar reset
    
    // Reinicializar
    return mpu6050_init(config);
}

int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t buffer[6];
    
    if (!mpu6050_read_reg(config, MPU6050_REG_ACCEL_XOUT_H, buffer, 6)) {
        return 0;
    }
    
    // Converter dados de 16 bits (big endian)
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z
    
    return 1;
}

int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t buffer[6];
    
    if (!mpu6050_read_reg(config, MPU6050_REG_GYRO_XOUT_H, buffer, 6)) {
        return 0;
    }
    
    // Converter dados de 16 bits (big endian)
    gyro[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    gyro[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y
    gyro[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z
    
    return 1;
}

int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t buffer[2];
    
    if (!mpu6050_read_reg(config, MPU6050_REG_TEMP_OUT_H, buffer, 2)) {
        return 0;
    }
    
    // Converter dados de 16 bits (big endian)
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    return 1;
}