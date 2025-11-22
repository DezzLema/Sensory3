#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <math.h>

// Настройки I2C для MPU6050 - правильные пины ESP32-H2
#define I2C_MASTER_SCL_IO           5  // D5
#define I2C_MASTER_SDA_IO           6  // D6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

// Адрес MPU6050
#define MPU6050_ADDR                 0x68
#define MPU6050_WHO_AM_I             0x75
#define MPU6050_PWR_MGMT_1           0x6B
#define MPU6050_ACCEL_XOUT_H         0x3B

// Настройки светодиодов - правильные пины ESP32-H2
#define LED_RING_PIN                 GPIO_NUM_8    // D8 - Кольцо
#define LED_STRIP_PIN                GPIO_NUM_3    // D3 - Отдельные NeoPixel
#define LED_RING_COUNT               16
#define LED_STRIP_COUNT              12
#define LED_RMT_RESOLUTION_HZ        10000000

static const char *TAG = "MEMS_LAB3";

// Структуры данных
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} mpu6050_data_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

// Глобальные переменные RMT
static rmt_channel_handle_t led_ring_chan = NULL;
static rmt_channel_handle_t led_strip_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// Инициализация I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Функции MPU6050
static esp_err_t mpu6050_register_write(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, 
                                     write_buf, sizeof(write_buf), 
                                     pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                       &reg_addr, 1, data, len, 
                                       pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_init(void) {
    // Выход из спящего режима
    ESP_ERROR_CHECK(mpu6050_register_write(MPU6050_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Проверка WHO_AM_I
    uint8_t data;
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I, &data, 1));
    
    if (data != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found, WHO_AM_I = 0x%02X", data);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

static esp_err_t mpu6050_read_accel(mpu6050_data_t *data) {
    uint8_t buffer[6];
    esp_err_t ret = mpu6050_register_read(MPU6050_ACCEL_XOUT_H, buffer, sizeof(buffer));
    
    if (ret == ESP_OK) {
        data->accel_x = (buffer[0] << 8) | buffer[1];
        data->accel_y = (buffer[2] << 8) | buffer[3];
        data->accel_z = (buffer[4] << 8) | buffer[5];
    }
    
    return ret;
}

// Инициализация RMT для светодиодов
static void led_strip_init(void) {
    // Конфигурация энкодера
    uint16_t t0h_ticks = (uint16_t)(0.3f * LED_RMT_RESOLUTION_HZ / 1000000);
    uint16_t t0l_ticks = (uint16_t)(0.9f * LED_RMT_RESOLUTION_HZ / 1000000);
    uint16_t t1h_ticks = (uint16_t)(0.9f * LED_RMT_RESOLUTION_HZ / 1000000);
    uint16_t t1l_ticks = (uint16_t)(0.3f * LED_RMT_RESOLUTION_HZ / 1000000);
    
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .duration0 = t0h_ticks,
            .level0 = 1,
            .duration1 = t0l_ticks,
            .level1 = 0
        },
        .bit1 = {
            .duration0 = t1h_ticks,
            .level0 = 1,
            .duration1 = t1l_ticks,
            .level1 = 0
        },
        .flags = {
            .msb_first = 1
        }
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder));
    
    // Канал для светодиодного кольца
    rmt_tx_channel_config_t ring_chan_config = {
        .gpio_num = LED_RING_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&ring_chan_config, &led_ring_chan));
    ESP_ERROR_CHECK(rmt_enable(led_ring_chan));
    
    // Канал для отдельных NeoPixel
    rmt_tx_channel_config_t strip_chan_config = {
        .gpio_num = LED_STRIP_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&strip_chan_config, &led_strip_chan));
    ESP_ERROR_CHECK(rmt_enable(led_strip_chan));
}

// Отправка данных на светодиоды
static void led_strip_set_pixels(rmt_channel_handle_t channel, rgb_color_t *colors, uint32_t num_leds) {
    uint8_t pixel_buffer[num_leds * 3];
    
    for (int i = 0; i < num_leds; i++) {
        pixel_buffer[i * 3 + 0] = colors[i].g;
        pixel_buffer[i * 3 + 1] = colors[i].r;
        pixel_buffer[i * 3 + 2] = colors[i].b;
    }
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(rmt_transmit(channel, led_encoder, pixel_buffer, sizeof(pixel_buffer), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, pdMS_TO_TICKS(100)));
}

// Функция для преобразования значения в цвет
static rgb_color_t value_to_color(int16_t value, int16_t min_val, int16_t max_val) {
    rgb_color_t color = {0, 0, 0};
    
    int normalized = (value - min_val) * 255 / (max_val - min_val);
    normalized = normalized < 0 ? 0 : (normalized > 255 ? 255 : normalized);
    
    if (normalized < 128) {
        color.b = 255 - normalized * 2;
        color.r = normalized * 2;
    } else {
        color.r = 255;
        color.g = (normalized - 128) * 2;
    }
    
    return color;
}

// Основная задача
static void tilt_indicator_task(void *pvParameters) {
    mpu6050_data_t accel_data;
    rgb_color_t ring_colors[LED_RING_COUNT];
    rgb_color_t strip_colors[LED_STRIP_COUNT];
    
    const int16_t TILT_THRESHOLD = 5000;
    
    // Калибровка
    if (mpu6050_read_accel(&accel_data) == ESP_OK) {
        int16_t base_x = accel_data.accel_x;
        int16_t base_y = accel_data.accel_y;
        
        ESP_LOGI(TAG, "Calibration complete: X=%d, Y=%d", base_x, base_y);

        while (1) {
            if (mpu6050_read_accel(&accel_data) == ESP_OK) {
                int16_t tilt_x = accel_data.accel_x - base_x;
                int16_t tilt_y = accel_data.accel_y - base_y;
                int16_t tilt_magnitude = abs(tilt_x) + abs(tilt_y);
                
                // Определение активного светодиода на кольце
                int active_led = -1;
                if (tilt_x < -TILT_THRESHOLD) active_led = 12;    // Влево
                else if (tilt_x > TILT_THRESHOLD) active_led = 4; // Вправо
                else if (tilt_y < -TILT_THRESHOLD) active_led = 0; // Вперед
                else if (tilt_y > TILT_THRESHOLD) active_led = 8;  // Назад
                
                // Обновление светодиодного кольца
                for (int i = 0; i < LED_RING_COUNT; i++) {
                    if (i == active_led) {
                        ring_colors[i] = (rgb_color_t){0, 0, 0}; // Погасший
                    } else {
                        ring_colors[i] = value_to_color(tilt_magnitude, 0, TILT_THRESHOLD * 2);
                    }
                }
                
                // Обновление отдельных NeoPixel (все одинаковые)
                rgb_color_t strip_color = value_to_color(tilt_magnitude, 0, TILT_THRESHOLD * 2);
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    strip_colors[i] = strip_color;
                }
                
                // Отправка данных на все светодиоды
                led_strip_set_pixels(led_ring_chan, ring_colors, LED_RING_COUNT);
                led_strip_set_pixels(led_strip_chan, strip_colors, LED_STRIP_COUNT);
                
                ESP_LOGI(TAG, "Tilt: X=%6d, Y=%6d, Mag=%6d, LED: %d", 
                        tilt_x, tilt_y, tilt_magnitude, active_led);
            } else {
                ESP_LOGE(TAG, "Failed to read accelerometer data");
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else {
        ESP_LOGE(TAG, "Failed to read accelerometer for calibration");
    }
    
    vTaskDelete(NULL);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Lab 3: MEMS Tilt Indicator with ESP32-H2");
    
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");
    
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed");
        return;
    }
    
    led_strip_init();
    ESP_LOGI(TAG, "LED strips initialized");
    
    xTaskCreate(tilt_indicator_task, "tilt_indicator", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "System ready");
}