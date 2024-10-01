#include "defs.h"

// Filas para Transmissão e Armazenamento de Dados
QueueHandle_t xAltQueue = NULL;

QueueHandle_t xLittleFSQueue = NULL;
QueueHandle_t xSDQueue = NULL;
QueueHandle_t xLoraQueue = NULL;

// Semáforos Mutex 
SemaphoreHandle_t xGPSMutex = NULL;
SemaphoreHandle_t xStatusMutex = NULL;

SemaphoreHandle_t xI2CMutex = NULL;

TaskHandle_t xTaskLora = NULL;
// Status
uint32_t STATUS = 0; 
int32_t tx_ready = pdTRUE;
// Objeto GPS usado na main e na aquisição de dados
gps_t gps;

