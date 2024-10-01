#include "defs.h"

void task_aquisicao(void *pvParameters);
void task_lora(void *pvParameters);
void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_verifica(void *pvParameters);

void app_main(void)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    // Create Mutexes 
    xGPSMutex = xSemaphoreCreateMutex();
    xStatusMutex = xSemaphoreCreateMutex();

    // Create Queues
    xAltQueue = xQueueCreate(2, sizeof(float));
    xLittleFSQueue = xQueueCreate(2, sizeof(dados));
    xSDQueue = xQueueCreate(2, sizeof(dados));
    xLoraQueue = xQueueCreate(2, sizeof(dados));

    // When initializing, blink LED and beep buzzer 10 times
    for (uint32_t i = 0; i < 7; i++)
    {
        gpio_set_level(BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(75));
        gpio_set_level(BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(75));
    }
    ESP_LOGI("Buzzer", "Inicializado");

    xTaskCreate(task_aquisicao, "Aquisicao", 6144, NULL, 3, NULL);
    xTaskCreate(task_lora, "Lora", 6144, NULL, 6, &xTaskLora);
    xTaskCreate(task_sd, "SD", 6144, NULL, 5, NULL);
    xTaskCreate(task_littlefs, "LittleFS", 6144, NULL, 5, NULL);
    xTaskCreate(task_verifica, "Verifica", 3072, NULL, 5, NULL);
}

void bmp_acquire(dados *data, bmp280_t *bmp280)
{
    float pressure, current_altitude, temp;
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
    // ESP_LOGI("BMP", "Pressure: %.2f Pa, Altitude: %.2f m, Temperature: %.2f °C", pressure, current_altitude, temp);
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

void task_aquisicao(void *pvParameters)
{

    xI2CMutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(i2cdev_init());

    // Inicializacao do BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.standby = BMP280_STANDBY_05; // Standby time 0.5ms
    bmp280_t dev_bmp;
    memset(&dev_bmp, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init_desc(&dev_bmp, BMP280_I2C_ADDRESS_0, 0, I2C_SDA, I2C_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev_bmp, &params));

    // // Inicializacao do MPU6050
    mpu6050_dev_t dev_mpu = {0};
    mpu6050_init_desc(&dev_mpu, ADDR, 0, I2C_SDA, I2C_SCL);
    ESP_ERROR_CHECK(mpu6050_init(&dev_mpu));
    mpu6050_set_full_scale_accel_range(&dev_mpu, MPU6050_ACCEL_RANGE_16); // Accelerometer range: +/- 16g
    mpu6050_set_dlpf_mode(&dev_mpu, MPU6050_DLPF_2);                      // Digital low pass filter: 2 (94Hz)
    ESP_LOGI("MPU", "Accel range: %d", dev_mpu.ranges.accel);
    ESP_LOGI("MPU", "Gyro range:  %d", dev_mpu.ranges.gyro);

    // Inicializacao do GPS
    gps_init();

    // Inicialização do ADC
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE};
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t adc_channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_4, &adc_channel);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);

    while (1)
    {
        dados data;

        // Aquisição de dados do barômetro
        bmp_acquire(&data, &dev_bmp);

        // Aquisição de dados do acelerômetro e giroscópio
        mpu6050_acquire(&data, &dev_mpu);

        // Aquisição de dados do GPS feita via NMEA Parser
        xSemaphoreTake(xGPSMutex, portMAX_DELAY);
        data.latitude = gps.latitude;
        data.longitude = gps.longitude;
        data.gps_altitude = gps.altitude;
        xSemaphoreGive(xGPSMutex);

        // Conversão ADC
        voltage_acquire(&data, &adc_handle, &cali_handle);

        // Update time e status
        data.time = esp_timer_get_time() / 1000;
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        // Print dos dados
        ESP_LOGI("Aquisição", "\tTime: %ld, Status: %ld, V: %.2f\r\n"
                              "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                              "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                              "\tGyro\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                              "\tGPS\t\tLat: %.5f, Lon: %.5f, Alt: %.2f",
                 data.time, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel.x, data.accel.y, data.accel.z,
                 data.rotation.x, data.rotation.y, data.rotation.z,
                 data.latitude, data.longitude, data.gps_altitude);

        // Envia para fila do SD, LittleFS e da verificação
        xQueueSend(xAltQueue, &data.bmp_altitude, 0);
        xQueueSend(xSDQueue, &data, 0); // Envia para fila do SD
        xQueueSend(xLittleFSQueue, &data, 0);
        
        static int n = 0;
        if (n++ % 10 == 0) // A cada 10 iterações, envia para fila do LoRa 
        {
            xQueueSend(xLoraQueue, &data, 0); // Send to LoRa queue
        }
        // Bitwise para Status
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// handle_interrupt_fromisr resumes handle_interrupt_task
void IRAM_ATTR handle_interrupt_fromisr(void *arg)
{
    xTaskResumeFromISR(xTaskLora);
}

// task_lora reads data from queue and sends it to Lora module
void task_lora(void *pvParameters)
{
    gpio_set_direction(E220_AUX, GPIO_MODE_INPUT);
    gpio_pullup_dis(E220_AUX);
    gpio_set_intr_type(E220_AUX, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(E220_AUX, handle_interrupt_fromisr, NULL);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, E220_TX, E220_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    dados data;

    while (1)
    {
        dados buffer[E220_BUFFER_SIZE / sizeof(dados)];
        // Read data from queue and put it in fifo
        for (int i = 0; i < E220_BUFFER_SIZE / sizeof(dados); i++)
        {
            xQueueReceive(xLoraQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }
        ESP_LOGI("LORA", "sending %d byte packet", E220_BUFFER_SIZE);
        uart_write_bytes(UART_NUM_2, (const void *)buffer, E220_BUFFER_SIZE);
        vTaskSuspend(NULL);
    }
}

void task_sd(void *pvParameters) // Código de teste verificar depois
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    ESP_LOGI("SD", "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.

    ESP_LOGI("SD", "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SD", "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI("SD", "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SD", "Failed to mount filesystem. "
                           "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE("SD", "Failed to initialize the card (%s). ",
                     esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI("SD", "Filesystem mounted");

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", mount_point, (long int)0);
    ESP_LOGI("SD", "Creating file %s", log_name);
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE("SD", "Failed to open file for writing");
    }
    fclose(f);

    while (1)
    {
        dados data;
        dados buffer[SD_BUFFER_SIZE / sizeof(dados)];

        // Read data from queue
        for (int i = 0; i < SD_BUFFER_SIZE / sizeof(dados); i++)
        {
            xQueueReceive(xSDQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }

        // Write buffer to file
        ESP_LOGE("SD", "Opening file for writing");
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE("SD", "Failed to open file for writing");
        }
        ESP_LOGE("SD", "Writing data to SD card");
        fwrite(buffer, sizeof(dados), SD_BUFFER_SIZE / sizeof(dados), f);
        fclose(f);
        ESP_LOGI("SD", "Data written to SD card");
    }
}
void task_littlefs(void *pvParameters)
{
    // LittleFS INIT
    ESP_LOGW("LittleFS", "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("LittleFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("LittleFS", "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE("LittleFS", "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE("LittleFS", "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGW("LittleFS", "Partition size: total: %d, used: %d", total, used);
    }

    // Create log file
    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", conf.base_path, (long int)0);
    ESP_LOGI("LittleFS", "Creating file %s", log_name);
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE("LittleFS", "Failed to open file for writing");
    }

    while (1)
    {
        dados data;
        dados buffer[LITTLEFS_BUFFER_SIZE / sizeof(dados)];

        // Read data from queue
        for (int i = 0; i < LITTLEFS_BUFFER_SIZE / sizeof(dados); i++)
        {
            xQueueReceive(xLittleFSQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE("LittleFS", "Failed to open file for writing");
        }
        fwrite(buffer, sizeof(dados), LITTLEFS_BUFFER_SIZE / sizeof(dados), f);
        fclose(f);
        used += sizeof(buffer);
        ESP_LOGI("LittleFS", "Data written to LittleFS.");
    }
}

void task_verifica(void *pvParameters)
{
    gpio_set_direction(DROGUE_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(MAIN_GPIO, GPIO_MODE_OUTPUT);

    float current_altitude = 0;
    float max_altitude = 0;
    float altitude_inicio = 0;

    xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
    altitude_inicio = current_altitude;

    while (1)
    {
        xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
        // Update max altitude
        if (current_altitude > max_altitude)
        {
            max_altitude = current_altitude;
        }
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (!(STATUS & DROGUE_ABERTO)) // If drogue não abriu
        {
            if (current_altitude < max_altitude - DROGUE_THRESHOLD) // se altitude atual for menor que a altitude máxima - threshold
            {
                STATUS |= DROGUE_ABERTO;
                xSemaphoreGive(xStatusMutex);
                gpio_set_level(DROGUE_GPIO, 1);
                gpio_set_level(BUZZER_GPIO, 1);
                ESP_LOGW("Verifica", "Drogue aberto");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(DROGUE_GPIO, 0);
            }
            else
                xSemaphoreGive(xStatusMutex);
        }
        else if (!(STATUS & MAIN_ABERTO)) // Se drogue abriu, verifica se main deve abrir
        {
            if (current_altitude < altitude_inicio + MAIN_ALTITUDE) // Se altitude atual for menor que a altitude inicial somada a altitude de abertura da main
            {
                STATUS |= MAIN_ABERTO;
                xSemaphoreGive(xStatusMutex);

                gpio_set_level(MAIN_GPIO, 1);
                ESP_LOGW("Verifica", "Main aberto");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(MAIN_GPIO, 0);

                // Delete task
                vTaskDelete(NULL);
            }
            else
                xSemaphoreGive(xStatusMutex);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}