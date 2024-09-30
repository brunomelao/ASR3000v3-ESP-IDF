#include "aquisicao.h"
#include "defs.h"


void bmp_acquire(dados *data, bmp280_t *bmp280)
{
    float pressure, current_altitude,temp;
    bmp280_read_float(bmp280, &temp, &pressure, NULL);
       
    current_altitude = 44330 * (1 - powf(pressure / 101325, 1 / 5.255));
 
    // Update max altitude
    if (current_altitude > data->max_altitude)
    {
        data->max_altitude = current_altitude;
    }
 
    data->bmp_altitude = current_altitude;
    data->pressure = pressure;
    data->temperature = temp;
    //ESP_LOGI("BMP", "Pressure: %.2f Pa, Altitude: %.2f m, Temperature: %.2f °C", pressure, current_altitude, temp);
}

void mpu6050_acquire(dados *data, mpu6050_dev_t *dev_mpu)
{

    ESP_ERROR_CHECK(mpu6050_get_motion(dev_mpu, &data->accel, &data->rotation)); 
    // Convert accel to m/s^2
    data->accel.x *= G;
    data->accel.y *= G;
    data->accel.z *= G;
    // ESP_LOGI("MPU", "Acceleration: x=%.4f   y=%.4f   z=%.4f", data->accel.x, data->accel.y, data->accel.z); 
    // ESP_LOGI("MPU", "Rotation:     x=%.4f   y=%.4f   z=%.4f", data->rotation.x, data->rotation.y, data->rotation.z);

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
void voltage_acquire(dados *data, adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle)
{
    int raw_voltage = 0;
    int voltage = 0;

    // Read voltage
    adc_oneshot_read(*adc_handle, ADC_CHANNEL_4, &raw_voltage);

    // Convert raw voltage to voltage
    adc_cali_raw_to_voltage(*cali_handle, raw_voltage, &voltage);

    // Update voltage
    data->voltage = (float)voltage * 2 / 1000;
}