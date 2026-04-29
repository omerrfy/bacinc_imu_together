/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TEKNOFEST Roket Aviyonik - Tam Hassasiyetli Kontrol Yazılımı
  * @author         : Ömer
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include <math.h>
#include <stdio.h>

/* --- Tanımlamalar --- */
#define BNO055_ADDR (0x28 << 1)
#define MS5611_ADDR (0x77 << 1)
#define I2C_TIMEOUT 20  // Sensör kilitlenirse ana döngünün asılı kalmasını önler

/* --- Değişkenler --- */
uint16_t C[7];
float ground_pressure = 1013.25f;
float pressure, temperature, altitude;
uint32_t D1, D2;

// IMU Verileri
float heading, roll, pitch;

typedef struct {
    int16_t euler_h, euler_r, euler_p;
} BNO_Data;
BNO_Data sensor_imu;

/* --- Fonksiyon Prototipleri --- */
void SystemClock_Config(void);
void MS5611_Reset(void);
void MS5611_Read_PROM(void);
void MS5611_Read_Raw(void);
void MS5611_Calculate(void);
void BNO055_Init(void);
void Error_Handler(void);

/**
  * @brief  Ana Fonksiyon
  */
int main(void)
{
  /* Donanım İlklendirme */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* 1. SENSÖR BAŞLATMA */
  MS5611_Reset();
  MS5611_Read_PROM();
  BNO055_Init();

  /* 2. YER BASINCI KALİBRASYONU (Rampada 50 örnek ortalama) */
  float p_sum = 0;
  int valid_samples = 0;
  for(int j = 0; j < 60; j++) {
      MS5611_Read_Raw();
      MS5611_Calculate();
      if(j >= 10) { // İlk verilerdeki dalgalanmayı atla
          p_sum += pressure;
          valid_samples++;
      }
      HAL_Delay(30);
  }
  if(valid_samples > 0) ground_pressure = p_sum / (float)valid_samples;

  /* 3. ANA DÖNGÜ (50 Hz) */
  while (1)
  {
    // --- MS5611 Barometre Okuma ---
    MS5611_Read_Raw();
    MS5611_Calculate();

    // --- BNO055 Euler Açıları Okuma ---
    uint8_t euler_buf[6];
    // Register 0x1A'dan başlayarak 6 byte oku (Heading, Roll, Pitch)
    if(HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x1A, 1, euler_buf, 6, I2C_TIMEOUT) == HAL_OK) {
        sensor_imu.euler_h = (int16_t)((euler_buf[1] << 8) | euler_buf[0]);
        sensor_imu.euler_r = (int16_t)((euler_buf[3] << 8) | euler_buf[2]);
        sensor_imu.euler_p = (int16_t)((euler_buf[5] << 8) | euler_buf[4]);

        // Dönüşüm: 1 Derece = 16 LSB
        heading = (float)sensor_imu.euler_h / 16.0f;
        roll    = (float)sensor_imu.euler_r / 16.0f;
        pitch   = (float)sensor_imu.euler_p / 16.0f;
    }

    /* BURAYA VERİ KAYIT VEYA TELEMETRİ FONKSİYONU GELEBİLİR */

    HAL_Delay(20);
  }
}

/* --- Sensör Fonksiyonları --- */

void BNO055_Init(void) {
    uint8_t op_mode = 0x0C; // NDOF modu (9 Eksen Füzyonu)
    // MOD register'ına yaz (0x3D)
    HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDR, 0x3D, 1, &op_mode, 1, I2C_TIMEOUT);
    HAL_Delay(100); // Sensörün moda geçmesi için bekleme
}

void MS5611_Reset(void) {
    uint8_t reset_cmd = 0x1E;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &reset_cmd, 1, I2C_TIMEOUT);
    HAL_Delay(15);
}

void MS5611_Read_PROM(void) {
    for(int i = 1; i < 7; i++) {
        uint8_t prom_addr = 0xA0 + (i * 2);
        uint8_t tmp[2];
        HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, prom_addr, 1, tmp, 2, I2C_TIMEOUT);
        C[i] = (uint16_t)((tmp[0] << 8) | tmp[1]);
    }
}

void MS5611_Read_Raw(void) {
    uint8_t raw_data[3];
    uint8_t start_d1 = 0x48; // Basınç OSR 4096
    uint8_t start_d2 = 0x58; // Sıcaklık OSR 4096
    uint8_t adc_read = 0x00;

    // Basınç Oku
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &start_d1, 1, I2C_TIMEOUT);
    HAL_Delay(10); // Dönüşüm süresi
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &adc_read, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, raw_data, 3, I2C_TIMEOUT);
    D1 = (uint32_t)((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]);

    // Sıcaklık Oku
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &start_d2, 1, I2C_TIMEOUT);
    HAL_Delay(10);
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &adc_read, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, raw_data, 3, I2C_TIMEOUT);
    D2 = (uint32_t)((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]);
}

void MS5611_Calculate(void) {
    // İlksel Fark ve Sıcaklık Hesaplama
    int32_t dT = D2 - (int32_t)C[5] * 256;
    int64_t TEMP = 2000 + (int64_t)dT * C[6] / 8388608;

    int64_t OFF = (int64_t)C[2] * 65536 + (int64_t)C[4] * dT / 128;
    int64_t SENS = (int64_t)C[1] * 32768 + (int64_t)C[3] * dT / 256;

    // --- İkinci Mertebe Sıcaklık Kompanzasyonu (T < 20C Durumu) ---
    if (TEMP < 2000) {
        int64_t T2 = (int64_t)dT * dT / 2147483648ULL;
        int64_t OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        int64_t SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;

        if (TEMP < -1500) { // Çok soğuk hava düzeltmesi
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    pressure = (float)(((D1 * SENS / 2097152) - OFF) / 32768) / 100.0f;
    temperature = (float)TEMP / 100.0f;

    // Standart Atmosferik Formül ile İrtifa Hesaplama
    if(pressure > 0 && ground_pressure > 0) {
        altitude = 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.190295f));
    }
}

/* --- Sistem Yapılandırması (ST'nin varsayılan değerleri) --- */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
