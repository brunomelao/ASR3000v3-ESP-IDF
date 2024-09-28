#include "defs.h"
#include "aquisicao.h"

void task_aquisicao(void *pvParameters);
void task_lora(void *pvParameters);
void task_sd(void *pvParameters);

void app_main(void)
{
    gpio_set_direction(RBF_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RBF_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

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

    xTaskCreate(task_aquisicao, "Aquisicao", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
    xTaskCreate(task_lora, "Lora", configMINIMAL_STACK_SIZE * 4, NULL, 6, &xTaskLora);
    xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
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
    // gps_init();

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
        data.latitude = -21.771706;  // gps.latitude;
        data.longitude = -43.381260; // gps.longitude;
        data.gps_altitude = 913.04;  // gps.altitude;
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

        // Filas dos dados
        if (!(data.status & LANDED)) // Se não pousou, envia para filas
        {
            if ((data.status & ARMED)) // Se armado, envia para fila de altitude
                xQueueSend(xAltQueue, &data.bmp_altitude, 0);
            xQueueSend(xSDQueue, &data, 0); // Envia para fila do SD
            if (!(data.status & LFS_FULL))  // Se LittleFS não está cheio, envia para LittleFSqueue
                xQueueSend(xLittleFSQueue, &data, 0);
        }
        static int n = 0;
        if (n++ % 10 == 0) // This affects the frequency of the LoRa messages
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
    }
}

void task_sd(void *pvParameters) // Código de teste verificar depois 
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
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
            // Check if landed
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            if (STATUS & LANDED)
            {
                xSemaphoreGive(xStatusMutex);
                ESP_LOGW("SD", "Landed, unmounting SD card");
                esp_vfs_fat_sdcard_unmount(mount_point, card);
                ESP_LOGI("SD", "Card unmounted");
                vTaskDelete(NULL); // Delete task
            }
            else
                xSemaphoreGive(xStatusMutex);

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