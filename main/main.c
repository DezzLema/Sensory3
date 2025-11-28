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

// Структура для авиагоризонта
typedef struct {
    float pitch;  // тангаж (наклон вперед/назад)
    float roll;   // крен (наклон влево/вправо)
} attitude_t;

// Глобальные переменные
static accel_type_t accel_data;
static attitude_t aircraft_attitude = {0, 0};
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

// Расчет углов наклона из данных акселерометра
static void calculate_attitude() {
    // Конвертация в g (ускорение свободного падения)
    float ax = accel_data.accel_x / 8192.0f;  // ±4g диапазон
    float ay = accel_data.accel_y / 8192.0f;
    float az = accel_data.accel_z / 8192.0f;
    
    // Расчет углов (в радианах)
    aircraft_attitude.roll = atan2f(ay, az);                    // Крен
    aircraft_attitude.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)); // Тангаж
    
    // Конвертация в градусы
    aircraft_attitude.roll = aircraft_attitude.roll * 180.0f / M_PI;
    aircraft_attitude.pitch = aircraft_attitude.pitch * 180.0f / M_PI;
    
    // Ограничение углов
    if (aircraft_attitude.roll > 45) aircraft_attitude.roll = 45;
    if (aircraft_attitude.roll < -45) aircraft_attitude.roll = -45;
    if (aircraft_attitude.pitch > 45) aircraft_attitude.pitch = 45;
    if (aircraft_attitude.pitch < -45) aircraft_attitude.pitch = -45;
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

// Отрисовка авиагоризонта с ОБРАТНОЙ индикацией
static void draw_reverse_artificial_horizon() {
    // Центр матрицы
    int center_x = LED_MATRIX_SIZE / 2;
    int center_y = LED_MATRIX_SIZE / 2;
    
    // ОБРАТНАЯ ИНДИКАЦИЯ: инвертируем углы
    float reverse_pitch = -aircraft_attitude.pitch; // Обратный тангаж
    float reverse_roll = -aircraft_attitude.roll;   // Обратный крен
    
    // Смещение горизонта - ОБРАТНАЯ индикация
    int horizon_offset = (int)(reverse_pitch * 2.0f); // Обратное смещение
    
    // Базовый горизонт (без наклона)
    int base_horizon = center_y + horizon_offset;
    
    // Отрисовка неба и земли с наклоном
    for (int y = 0; y < LED_MATRIX_SIZE; y++) {
        for (int x = 0; x < LED_MATRIX_SIZE; x++) {
            // Расчет наклона горизонта (ОБРАТНАЯ индикация)
            int tilted_horizon = base_horizon + (int)(reverse_roll * (x - center_x) * 0.2f);
            
            // Определение цвета: выше наклоненного горизонта - земля, ниже - небо
            if (y < tilted_horizon) {
                // ЗЕМЛЯ (коричневая) - СВЕРХУ при обратной индикации
                set_pixel_color(x, y, 120, 60, 0); // Коричневый
            } else {
                // НЕБО (синее) - СНИЗУ при обратной индикации
                set_pixel_color(x, y, 0, 0, 150); // Синий
            }
        }
    }
    
    // Отрисовка линии горизонта
    for (int x = 0; x < LED_MATRIX_SIZE; x++) {
        int horizon_y = base_horizon + (int)(reverse_roll * (x - center_x) * 0.2f);
        
        if (horizon_y >= 0 && horizon_y < LED_MATRIX_SIZE) {
            set_pixel_color(x, horizon_y, 255, 255, 255); // Белая линия горизонта
        }
    }
    
    // Центральный указатель (самолет) - всегда в центре
    set_pixel_color(center_x, center_y, 255, 0, 0); // Красный центр
    
    // Крылья самолета (горизонтальные)
    for (int dx = -2; dx <= 2; dx++) {
        if (dx != 0) { // Пропускаем центр
            int wing_x = center_x + dx;
            if (wing_x >= 0 && wing_x < LED_MATRIX_SIZE) {
                set_pixel_color(wing_x, center_y, 255, 255, 255);
            }
        }
    }
    
    // Визуальные индикаторы наклона по краям матрицы
    // Индикатор тангажа слева
    int pitch_indicator = center_y + (int)(aircraft_attitude.pitch * 0.5f);
    for (int dy = -1; dy <= 1; dy++) {
        int indicator_y = pitch_indicator + dy;
        if (indicator_y >= 0 && indicator_y < LED_MATRIX_SIZE) {
            set_pixel_color(0, indicator_y, 0, 255, 0); // Зеленый - тангаж
        }
    }
    
    // Индикатор крена сверху
    int roll_indicator = center_x + (int)(aircraft_attitude.roll * 0.5f);
    for (int dx = -1; dx <= 1; dx++) {
        int indicator_x = roll_indicator + dx;
        if (indicator_x >= 0 && indicator_x < LED_MATRIX_SIZE) {
            set_pixel_color(indicator_x, 0, 255, 255, 0); // Желтый - крен
        }
    }
}

// Задача для работы с MPU6050
static void mpu6050_task(void *pvParameter) {
    while (true) {
        if (mpu6050_read_accel(&accel_data) == ESP_OK) {
            calculate_attitude();
            printf("Accel: X=%d, Y=%d, Z=%d | Pitch=%.1f°, Roll=%.1f°\n", 
                   accel_data.accel_x, accel_data.accel_y, accel_data.accel_z,
                   aircraft_attitude.pitch, aircraft_attitude.roll);
        } else {
            printf("Error reading accel data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача для анимации авиагоризонта
static void horizon_task(void *pvParameter) {
    while (true) {
        // Очистка и отрисовка
        clear_matrix();
        draw_reverse_artificial_horizon();
        update_matrix();
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
    }
}

void app_main(void) {
    // Инициализация периферии
    i2c_init();
    mpu6050_init();
    ws2812_init();
    
    printf("Reverse Artificial Horizon Demo Started\n");
    printf("Variant 4: Reverse indication - horizon moves OPPOSITE to tilt\n");
    printf("Tilt FORWARD -> horizon goes UP\n");
    printf("Tilt BACK -> horizon goes DOWN\n");
    printf("Tilt RIGHT -> horizon tilts LEFT\n");
    printf("Tilt LEFT -> horizon tilts RIGHT\n");
    
    // Создание задач
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 2, NULL);
    xTaskCreate(horizon_task, "horizon_task", 4096, NULL, 2, NULL);
    
    // Бесконечный цикл
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}