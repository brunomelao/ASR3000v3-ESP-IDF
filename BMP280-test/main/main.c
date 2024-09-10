#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <string.h>
#include <math.h>
#define BMP_SCL 9
#define BMP_SDA 8

void bmp280_test(void *pvParameters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, BMP_SDA, BMP_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    float pressure, temperature, altitude;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (bmp280_read_float(&dev, &temperature, &pressure, NULL) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }
        altitude = 44330 * (1 - powf(pressure / 101325, 1/5.255));
        printf("Pressure: %.2f Pa, Temperature: %.2f C, Altitude: %.2f\n", pressure, temperature, altitude);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
