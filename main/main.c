#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 40000
#define I2C_MASTER_NUM I2C_NUM_0  

// WS2812
#define LED_GPIO 3
#define LED_NUM 64
#define LED_MATRIX_SIZE 8

// Акселерометр MPU6050
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} accel_type_t;

// Глобальные переменные
static accel_type_t accel_data;
static uint8_t led_matrix[LED_NUM * 3];
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// Инициализация I2C
static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Запись в регистр MPU6050
static esp_err_t mpu6050_write_reg(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = {addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x68, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

// Чтение регистров MPU6050
static esp_err_t mpu6050_read_reg(uint8_t addr, uint8_t *data, uint8_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &addr, 1, data, len, pdMS_TO_TICKS(1000));
}

// Инициализация MPU6050
static void mpu6050_init() {
    mpu6050_write_reg(0x6B, 0x00); // Выход из спящего режима
    mpu6050_write_reg(0x1C, 0x08); // Акселерометр: ±4g
}

// Чтение данных акселерометра
static esp_err_t mpu6050_read_accel(accel_type_t *data) {
    uint8_t buf[6];
    esp_err_t err = mpu6050_read_reg(0x3B, buf, sizeof(buf));

    data->accel_x = (buf[0] << 8) | buf[1];
    data->accel_y = (buf[2] << 8) | buf[3];
    data->accel_z = (buf[4] << 8) | buf[5];

    return err;
}

// Расчет наклона из данных акселерометра
static void calculate_tilt(float *tilt_x, float *tilt_y) {
    // Конвертация в g
    float ax = accel_data.accel_x / 8192.0f;
    float ay = accel_data.accel_y / 8192.0f;
    float az = accel_data.accel_z / 8192.0f;
    
    // Расчет углов наклона
    *tilt_x = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI; // Наклон влево/вправо
    *tilt_y = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI; // Наклон вперед/назад
    
    // Ограничение
    if (*tilt_x > 45) *tilt_x = 45;
    if (*tilt_x < -45) *tilt_x = -45;
    if (*tilt_y > 45) *tilt_y = 45;
    if (*tilt_y < -45) *tilt_y = -45;
}

// Инициализация RMT для WS2812
static void ws2812_init() {
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 10000000,
        .trans_queue_depth = 4,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,
            .level1 = 0,
            .duration1 = 3,
        },
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

// Установка цвета пикселя в матрице
static void set_pixel_color(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x >= LED_MATRIX_SIZE || y >= LED_MATRIX_SIZE) return;
    
    uint16_t index;
    if (y % 2 == 0) {
        index = y * LED_MATRIX_SIZE + x;
    } else {
        index = y * LED_MATRIX_SIZE + (LED_MATRIX_SIZE - 1 - x);
    }
    
    led_matrix[index * 3 + 0] = g;
    led_matrix[index * 3 + 1] = r;
    led_matrix[index * 3 + 2] = b;
}

// Очистка матрицы
static void clear_matrix() {
    memset(led_matrix, 0, sizeof(led_matrix));
}

// Обновление LED матрицы
static void update_matrix() {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_matrix, sizeof(led_matrix), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Индикация наклона для оценки 3 с использованием всей матрицы
static void draw_tilt_indication() {
    float tilt_x, tilt_y;
    calculate_tilt(&tilt_x, &tilt_y);
    
    // Определяем зону, которая должна гаснуть
    int dead_zone_start_x = -1, dead_zone_end_x = -1;
    int dead_zone_start_y = -1, dead_zone_end_y = -1;
    
    // Определяем какая зона гаснет в зависимости от наклона
    float threshold = 10.0f; // Порог наклона для гашения
    
    if (tilt_x > threshold) {
        // Наклон вправо - гаснет правая часть
        dead_zone_start_x = LED_MATRIX_SIZE / 2;
        dead_zone_end_x = LED_MATRIX_SIZE - 1;
        dead_zone_start_y = 0;
        dead_zone_end_y = LED_MATRIX_SIZE - 1;
    } else if (tilt_x < -threshold) {
        // Наклон влево - гаснет левая часть
        dead_zone_start_x = 0;
        dead_zone_end_x = LED_MATRIX_SIZE / 2 - 1;
        dead_zone_start_y = 0;
        dead_zone_end_y = LED_MATRIX_SIZE - 1;
    } else if (tilt_y > threshold) {
        // Наклон вперед - гаснет верхняя часть
        dead_zone_start_x = 0;
        dead_zone_end_x = LED_MATRIX_SIZE - 1;
        dead_zone_start_y = 0;
        dead_zone_end_y = LED_MATRIX_SIZE / 2 - 1;
    } else if (tilt_y < -threshold) {
        // Наклон назад - гаснет нижняя часть
        dead_zone_start_x = 0;
        dead_zone_end_x = LED_MATRIX_SIZE - 1;
        dead_zone_start_y = LED_MATRIX_SIZE / 2;
        dead_zone_end_y = LED_MATRIX_SIZE - 1;
    }
    
    // Базовый цвет в зависимости от направления наклона
    float total_tilt = sqrtf(tilt_x*tilt_x + tilt_y*tilt_y);
    uint8_t base_intensity = (uint8_t)(fabsf(total_tilt) * 5.0f);
    if (base_intensity > 255) base_intensity = 255;
    
    // Цвет зависит от направления наклона
    uint8_t r, g, b;
    if (fabsf(tilt_x) > fabsf(tilt_y)) {
        // Преобладает горизонтальный наклон
        if (tilt_x > 0) {
            r = base_intensity; g = base_intensity/2; b = 0; // Оранжевый для правого наклона
        } else {
            r = 0; g = base_intensity; b = base_intensity/2; // Бирюзовый для левого наклона
        }
    } else {
        // Преобладает вертикальный наклон
        if (tilt_y > 0) {
            r = base_intensity; g = 0; b = base_intensity/2; // Пурпурный для наклона вперед
        } else {
            r = base_intensity/2; g = base_intensity; b = 0; // Желто-зеленый для наклона назад
        }
    }
    
    // Отрисовка всей матрицы 8x8
    for (int y = 0; y < LED_MATRIX_SIZE; y++) {
        for (int x = 0; x < LED_MATRIX_SIZE; x++) {
            // Проверяем, находится ли пиксель в гаснущей зоне
            bool is_dead_zone = (dead_zone_start_x != -1) && 
                               (x >= dead_zone_start_x && x <= dead_zone_end_x) &&
                               (y >= dead_zone_start_y && y <= dead_zone_end_y);
            
            if (is_dead_zone) {
                // Гаснущая зона - черный цвет
                set_pixel_color(x, y, 0, 0, 0);
            } else {
                // Активная зона - цвет зависит от наклона
                // Создаем градиент от центра к краям
                float center_x = (LED_MATRIX_SIZE - 1) / 2.0f;
                float center_y = (LED_MATRIX_SIZE - 1) / 2.0f;
                float distance = sqrtf(powf(x - center_x, 2) + powf(y - center_y, 2));
                float max_distance = sqrtf(powf(center_x, 2) + powf(center_y, 2));
                float intensity_factor = 1.0f - (distance / max_distance) * 0.5f;
                
                uint8_t pixel_r = (uint8_t)(r * intensity_factor);
                uint8_t pixel_g = (uint8_t)(g * intensity_factor);
                uint8_t pixel_b = (uint8_t)(b * intensity_factor);
                
                set_pixel_color(x, y, pixel_r, pixel_g, pixel_b);
            }
        }
    }
    
    // Центральный крест для ориентации
    for (int i = 0; i < LED_MATRIX_SIZE; i++) {
        set_pixel_color(LED_MATRIX_SIZE/2, i, 255, 255, 255); // Вертикальная линия
        set_pixel_color(i, LED_MATRIX_SIZE/2, 255, 255, 255); // Горизонтальная линия
    }
}

// Задача для работы с MPU6050
static void mpu6050_task(void *pvParameter) {
    while (true) {
        if (mpu6050_read_accel(&accel_data) == ESP_OK) {
            float tilt_x, tilt_y;
            calculate_tilt(&tilt_x, &tilt_y);
            printf("Accel: X=%d, Y=%d, Z=%d | Tilt: X=%.1f°, Y=%.1f°\n", 
                   accel_data.accel_x, accel_data.accel_y, accel_data.accel_z,
                   tilt_x, tilt_y);
        } else {
            printf("Error reading accel data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача для индикации наклона
static void tilt_indication_task(void *pvParameter) {
    while (true) {
        // Очистка и отрисовка
        clear_matrix();
        draw_tilt_indication();
        update_matrix();
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 FPS
    }
}

void app_main(void) {
    // Инициализация периферии
    i2c_init();
    mpu6050_init();
    ws2812_init();
    
    printf("MEMS Accelerometer Tilt Indication Demo Started\n");
    printf("Variant 4 (Grade 3): Full 8x8 matrix tilt indication\n");
    printf("Dead zones: Right(tilt>10°), Left(tilt<-10°), Top(tilt>10°), Bottom(tilt<-10°)\n");
    
    // Создание задач
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 2, NULL);
    xTaskCreate(tilt_indication_task, "tilt_task", 4096, NULL, 2, NULL);
    
    // Бесконечный цикл
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}