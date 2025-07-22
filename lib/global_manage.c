#include "global_manage.h" // Inclusão do cabeçalho do arquivo
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "aht20.h"      // Biblioteca do AHT20
#include "bmp280.h"     // Biblioteca do BMP280
#include <math.h>

// Defines do Sensor
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

// Variáveis de Estado Globais
static SENSOR_DATA g_sensor_data;       // Estrutura que armazena todos os dados e configurações.
static struct bmp280_calib_param bmp_params;    // Parâmetros de calibração do BMP280, lidos uma vez na inicialização.


void ler_sensores() { 
    // Leitura do Sensor BMP280 
    int32_t raw_temp_bmp, raw_pressure;
    bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure); 
    
    // Converte os valores brutos para unidades legíveis
    int32_t temp_converted = bmp280_convert_temp(raw_temp_bmp, &bmp_params); 
    int32_t press_converted = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &bmp_params); 

    // Aplica todos os offsets
    g_sensor_data.temperatura_bmp = (temp_converted / 100.0f) + g_sensor_data.offset_temp;
    g_sensor_data.pressao_hpa = (press_converted / 100.0f) + g_sensor_data.offset_press;
    double altitude_calculada = 44330.0 * (1.0 - pow(press_converted / SEA_LEVEL_PRESSURE, 0.1903));
    g_sensor_data.altitude = altitude_calculada + g_sensor_data.offset_alt; 

    // Leitura do Sensor AHT20 
    AHT20_Data aht_data;
    if (aht20_read(I2C_PORT, &aht_data)) {
        g_sensor_data.umidade_aht = aht_data.humidity + g_sensor_data.offset_umid; // APLICA OFFSET DE UMIDADE
    }

    // Desloca todos os valores que já estão no vetor uma posição à esqueda
    for (int i = 0; i < 19; i++) {
        g_sensor_data.hist_temp[i] = g_sensor_data.hist_temp[i + 1];
        g_sensor_data.hist_umid[i] = g_sensor_data.hist_umid[i + 1];
        g_sensor_data.hist_press[i] = g_sensor_data.hist_press[i + 1];
        g_sensor_data.hist_alt[i] = g_sensor_data.hist_alt[i + 1];
    }

    // Adiciona os novos valores na última posição de cada vetor
    g_sensor_data.hist_temp[19] = g_sensor_data.temperatura_bmp;
    g_sensor_data.hist_umid[19] = g_sensor_data.umidade_aht;
    g_sensor_data.hist_press[19] = g_sensor_data.pressao_hpa;
    g_sensor_data.hist_alt[19] = g_sensor_data.altitude;
}


void setup_sensor_manager(void) {
    // Inicializa o hardware I2C
    i2c_init(I2C_PORT, 400 * 1000); 
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); 
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C); 
    gpio_pull_up(I2C_SDA_PIN); 
    gpio_pull_up(I2C_SCL_PIN); 

    // Inicializa os sensores
    bmp280_init(I2C_PORT); 
    bmp280_get_calib_params(I2C_PORT, &bmp_params); 
    aht20_reset(I2C_PORT); 
    aht20_init(I2C_PORT); 
    
    // Zera os valores iniciais na estrutura de dados
    memset(&g_sensor_data, 0, sizeof(SENSOR_DATA));

    // Definição de limites mínimos e máximos padrão
    g_sensor_data.limite_min_temp = 20; 
    g_sensor_data.limite_max_temp = 40; 
    g_sensor_data.limite_min_umid = 30;
    g_sensor_data.limite_max_umid = 65;
    g_sensor_data.limite_min_press = 950;
    g_sensor_data.limite_max_press = 1200;
    g_sensor_data.limite_min_alt = 0;
    g_sensor_data.limite_max_alt = 1500;

}

SENSOR_DATA* get_sensor_data(void) {
    return &g_sensor_data;
}

void setar_temp_offset(float offset) {
    g_sensor_data.offset_temp = offset;
}

void setar_press_offset(float offset) {
    g_sensor_data.offset_press = offset;
}

void setar_umid_offset(float offset) {
    g_sensor_data.offset_umid = offset;
}

void setar_alt_offset(float offset) {
    g_sensor_data.offset_alt = offset;
}

void setar_limites_temp(int min, int max) {
    g_sensor_data.limite_min_temp = min;
    g_sensor_data.limite_max_temp = max;
}

void setar_limites_umid(int min, int max) {
    g_sensor_data.limite_min_umid = min;
    g_sensor_data.limite_max_umid = max;
}

void setar_limites_press(int min, int max) {
    g_sensor_data.limite_min_press = min;
    g_sensor_data.limite_max_press = max;
}

void setar_limites_alt(int min, int max) {
    g_sensor_data.limite_min_alt = min;
    g_sensor_data.limite_max_alt = max;
}