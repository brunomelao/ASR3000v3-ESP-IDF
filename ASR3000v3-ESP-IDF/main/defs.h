#ifndef DEFS_H
#define DEFS_H

// Verificar a necessidade de incluir os headers abaixo
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

//Sensores
#include "bmp280.h"
#include "nmea_parser.h"
#include "mpu6050.h"
#include "i2cdev.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
// Include libraries for SD card 
#include "sys/stat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unistd.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"

#include "esp_littlefs.h"

#define BUZZER_GPIO 38
#define GPS_RX 14
#define GPS_TX 21
#define I2C_SCL 9
#define I2C_SDA 8
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define SD_CS 10
#define SD_BUFFER_SIZE 512
#define LITTLEFS_BUFFER_SIZE 512
#define E220_RX 36
#define E220_TX 35
#define E220_AUX 37
#define E220_BUFFER_SIZE 192 
#define DROGUE_GPIO 48
#define MAIN_GPIO 47

#define TIME_ZONE (-3)   //Beijing Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

// Status flags
#define DROGUE_ABERTO (1 << 0)
#define MAIN_ABERTO (1 << 1)

#define G 9.80665

#define DROGUE_THRESHOLD 20
#define MAIN_ALTITUDE 500
typedef struct // size = ? bytes
{
    int32_t time;
    uint32_t status;

    float pressure;
    float temperature;
    float bmp_altitude;
    float max_altitude;

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    float latitude;
    float longitude;
    float gps_altitude;
    float voltage;
} dados;


#define CONFIG_MPU6050_I2C_ADDRESS_LOW
// MPU6050
#ifdef CONFIG_MPU6050_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

// Filas para Transmissão e Armazenamento de Dados
extern QueueHandle_t xAltQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLoraQueue;

// Semáforos Mutex
extern SemaphoreHandle_t xGPSMutex;
extern SemaphoreHandle_t xStatusMutex;
extern SemaphoreHandle_t xI2CMutex;

extern TaskHandle_t xTaskLora;

// Status
extern uint32_t STATUS;
// Lora tx ready
extern int32_t tx_ready;

// Objeto GPS usado na main e na aquisição de dados
extern gps_t gps;

#endif // DEFS_H