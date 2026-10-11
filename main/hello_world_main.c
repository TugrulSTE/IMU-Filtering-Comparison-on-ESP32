#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- I2C AYARLARI ---
#define I2C_MASTER_SCL_IO           14      
#define I2C_MASTER_SDA_IO           13      
#define I2C_MASTER_NUM              I2C_NUM_0 
#define MPU6500_ADDR                0x68    
#define MPU6500_PWR_MGMT_1          0x6B
#define MPU6500_ACCEL_XOUT_H        0x3B

// --- KALMAN FİLTRESİ DEĞİŞKENLERİ ---
/* * NASIL AYARLANIR?
 * R_angle (Ölçüm Gürültüsü): Sensörünüz çok titriyorsa bunu artırın (Örn: 0.5). 
 * Çizgi pürüzsüzleşir ama gecikme artar.
 * Q_angle (Süreç Gürültüsü): Sistemin tepki hızıdır. Ani hareketleri yakalamak için artırın.
 * Çok artarsa filtreleme özelliği azalır.
 */
float Q_angle = 0.15; // Süreç Gürültüsü (Hassasiyet)
float R_angle = 0.4;  // Ölçüm Gürültüsü (Gürültü miktarı)
float P_00 = 1.0;     // Hata Kovaryansı (Başlangıç)
float K_00 = 0.0;     // Kalman Kazancı
float x_estimate = 0.0; // En son tahmin edilen (filtrelenmiş) değer

// --- KALMAN FONKSİYONU ---
float update_kalman(float measurement) {
    // 1. Tahmin Aşaması (Prediction)
    P_00 = P_00 + Q_angle;

    // 2. Güncelleme Aşaması (Update)
    K_00 = P_00 / (P_00 + R_angle); // Kazanç (Gain) hesabı
    x_estimate = x_estimate + K_00 * (measurement - x_estimate); // Tahmini güncelle
    P_00 = (1 - K_00) * P_00; // Hatayı güncelle

    return x_estimate;
}

// --- SMA (Simple Moving Average) ---
#define SMA_SIZE 10
float sma_buffer[SMA_SIZE] = {0};
int sma_index = 0;

float get_sma(float new_sample) {
    sma_buffer[sma_index] = new_sample;
    sma_index = (sma_index + 1) % SMA_SIZE;
    float sum = 0;
    for(int i=0; i < SMA_SIZE; i++) sum += sma_buffer[i];
    return sum / SMA_SIZE;
}

// --- EMA (Exponential Moving Average) ---
float filtered_y_ema = 0.0;
float filtered_y_ema2 = 0.0;
const float alpha = 0.4; // EMA katsayısı
const float alpha2 = 0.8; // EMA katsayısı

esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6500_init() {
    uint8_t data[] = {MPU6500_PWR_MGMT_1, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_ADDR, data, sizeof(data), pdMS_TO_TICKS(100));
    printf("MPU6500 Baslatildi.\n");
}

// --- ANA SENSÖR TASKI ---
void mpu_task(void *pvParameters) {
    uint8_t data[6];
    int16_t raw_y;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    x_estimate = 0.0; 

    while (1) {
        uint8_t reg_addr = MPU6500_ACCEL_XOUT_H;
        if (i2c_master_write_read_device(I2C_MASTER_NUM, MPU6500_ADDR, &reg_addr, 1, data, 6, pdMS_TO_TICKS(50)) == ESP_OK) {
            
            raw_y = (int16_t)((data[2] << 8) | data[3]);
            float acc_y_g = (float)raw_y / 16384.0; // Ham G değeri

            filtered_y_ema = (alpha * acc_y_g) + (1.0 - alpha) * filtered_y_ema;
            filtered_y_ema2 = (alpha2 * acc_y_g) + (1.0 - alpha2) * filtered_y_ema2;

            float filtered_y_sma = get_sma(acc_y_g);

            float filtered_y_kalman = update_kalman(acc_y_g);

            printf("Ham Veri:%.3f,SMA:%.3f,EMA(0.4):%.3f,EMA(0.8):%.3f,Kalman:%.3f\n", 
                   acc_y_g, filtered_y_sma, filtered_y_ema, filtered_y_ema2 ,filtered_y_kalman);
        }
        else {
            printf("Sensor Hatasi!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz (Saniyede 50 veri)
    }
}

void app_main(void) {
    printf("Sistem ve Filtre Testi Baslatiliyor...\n");
    i2c_master_init();
    mpu6500_init();

    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, NULL);
}