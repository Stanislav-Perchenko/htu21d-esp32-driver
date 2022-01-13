/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "htu21d.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#define HTU21D_SDA_GPIO 26
#define HTU21D_SCL_GPIO 27
#define HTU21D_I2C_NUM  I2C_NUM_0

#define DELAY_SENSOR_READ_MS 1000



typedef struct {
    uint8_t task_index;
    htu21d_resolution_t sensor_resolution;
} demo_task_argument_t;




SemaphoreHandle_t htu_mux = NULL;





static void htu21d_test_task(void *arg)
{
    demo_task_argument_t* task_params = (demo_task_argument_t*)arg;
    htu21d_result_t htu_result;
    uint32_t task_idx = (uint32_t)(task_params->task_index);
    uint8_t temp_resolution;
    uint8_t rh_resolution;

    float temperature;
    float rh;

    uint32_t cnt = 0;

    htu21dGetResolutionValuesFromType(task_params->sensor_resolution, &temp_resolution, &rh_resolution);

    while (1) {
        xSemaphoreTake(htu_mux, portMAX_DELAY);
        printf("TASK[%d] test cnt: %d\r\n", task_idx, cnt++);

        printf("Setting up sensor resolutions (T -> %dbit, RH -> %dbit)... ", temp_resolution, rh_resolution);
        htu_result = htu21dSetResolution(task_params->sensor_resolution);
        if (htu_result == HTU21D_OK) {
            printf("OK.\r\n");
        } else {
            printf("Error - %d\r\n", htu_result);
        }

        printf("Reading temperature... ");
        htu_result = htu21dReadTemperature(&temperature);
        if (htu_result == HTU21D_OK) {
            printf("OK: %.2f\r\n", temperature);
        } else {
            printf("Error - %d\r\n", htu_result);
        }

        printf("Reading humidity... ");
        htu_result = htu21dReadHumidity(&rh, (htu_result == HTU21D_OK) ? htu21d_humidity_compensated : htu21d_humidity_non_compensated, temperature);
        if (htu_result == HTU21D_OK) {
            printf("OK: %.1f\r\n", rh);
        } else {
            printf("Error - %d\r\n", htu_result);
        }

        printf("\r\n\n\n");

        xSemaphoreGive(htu_mux);

        vTaskDelay((DELAY_SENSOR_READ_MS * (task_idx)) / portTICK_RATE_MS);
    }
    vSemaphoreDelete(htu_mux);
    vTaskDelete(NULL);
}


void app_main(void)
{

    htu21d_result_t htu_result;
    demo_task_argument_t task1_params = {
        .task_index = 1,
        .sensor_resolution = htu21d_resolution_t14b_rh12b
    };

    demo_task_argument_t task2_params = {
        .task_index = 2,
        .sensor_resolution = htu21d_resolution_t13b_rh10b
    };

    demo_task_argument_t task3_params = {
        .task_index = 3,
        .sensor_resolution = htu21d_resolution_t12b_rh8b
    };

    demo_task_argument_t task4_params = {
        .task_index = 4,
        .sensor_resolution = htu21d_resolution_t11b_rh11b
    };

    printf("Initializing HTU21D driver... ");

    htu_result = htu21dInit(HTU21D_I2C_NUM, HTU21D_SDA_GPIO, HTU21D_SCL_GPIO, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE);
    if (htu_result == HTU21D_OK) {
        printf("OK.\n");
    } else {
        printf("Error - %d", htu_result);
    }

    printf("Check HTU21D sensor is presented... ");
    htu_result = htu21dCheckPresent();
    if (htu_result == HTU21D_OK) {
        printf("OK.\n");

        htu_mux = xSemaphoreCreateMutex();
        xTaskCreate(htu21d_test_task, "htu21d_task_1", 1024 * 2, (void *)(&task1_params), 10, NULL);
        xTaskCreate(htu21d_test_task, "htu21d_task_2", 1024 * 2, (void *)(&task2_params), 10, NULL);
        xTaskCreate(htu21d_test_task, "htu21d_task_3", 1024 * 2, (void *)(&task3_params), 10, NULL);
        xTaskCreate(htu21d_test_task, "htu21d_task_4", 1024 * 2, (void *)(&task4_params), 10, NULL);

        while(1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        printf("Not found - %d\r\n", htu_result);
        for (int i = 10; i >= 0; i--) {
            printf("Restarting in %d seconds...\r\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("Restarting now.\r\n");
        fflush(stdout);
        esp_restart();
    }

}
