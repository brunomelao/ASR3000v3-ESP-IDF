#include <stdio.h>
#include "nmea_parser.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define GPS_RX 14
#define GPS_TX 21
#define TIME_ZONE -3
#define YEAR_BASE 2000

typedef struct // size = ? bytes
{
    float latitude;
    float longitude;
    float gps_altitude;
} dados;

SemaphoreHandle_t xGPSMutex = NULL;
gps_t gps;
void task_aquisicao(void *pvParameters);

void app_main(void)
{
    
    
    // Create Mutexes
    xGPSMutex = xSemaphoreCreateMutex();
    
    
    xTaskCreate(task_aquisicao, "Aquisicao", 1024 * 4, NULL, 3, NULL);
    
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case GPS_UPDATE:
        xSemaphoreTake(xGPSMutex, portMAX_DELAY);
        gps = *(gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI("GPS", "%d/%d/%d %d:%d:%d => \r\n"
                        "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                            "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                        "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                        "\t\t\t\t\t\tspeed      = %fm/s",
                 gps.date.year + YEAR_BASE, gps.date.month, gps.date.day,
                 gps.tim.hour + TIME_ZONE, gps.tim.minute, gps.tim.second,
                 gps.latitude, gps.longitude, gps.altitude, gps.speed);
        
        xSemaphoreGive(xGPSMutex);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW("GPS", "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}
void gps_init(void)
{
    // Inicializa o GPS
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT(); // NMEA parser configuration
    config.uart.baud_rate = 115200;
    config.uart.rx_pin = GPS_RX;
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);  // init NMEA parser library
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL); // register event handler for NMEA parser library
}
void task_aquisicao(void *pvParameters)
{
    gps_init();
    while (1)
    {
        dados data;
        // Aquisição de dados do GPS feita via NMEA Parser
        xSemaphoreTake(xGPSMutex, portMAX_DELAY);
        data.latitude = gps.latitude;
        data.longitude = gps.longitude;
        data.gps_altitude = gps.altitude;
        ESP_LOGI("GPS", "Latitude: %.05f°N, Longitude: %.05f°E, Altitude: %.02fm", data.latitude, data.longitude, data.gps_altitude);
        xSemaphoreGive(xGPSMutex);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

