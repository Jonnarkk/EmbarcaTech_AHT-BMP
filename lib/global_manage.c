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
static SENSOR_DATA dados_sensor_global;       // Estrutura que armazena todos os dados e configurações.
static struct bmp280_calib_param bmp_params;    // Parâmetros de calibração do BMP280, lidos uma vez na inicialização.


void ler_sensores() { 
    // Leitura do Sensor BMP280 
    int32_t raw_temp_bmp, raw_pressure;
    bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure); 
    
    // Converte os valores brutos para unidades legíveis
    int32_t temp_converted = bmp280_convert_temp(raw_temp_bmp, &bmp_params); 
    int32_t press_converted = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &bmp_params); 

    // Aplica todos os offsets
    dados_sensor_global.temperatura_bmp = (temp_converted / 100.0f) + dados_sensor_global.offset_temp;
    dados_sensor_global.pressao_hpa = (press_converted / 100.0f) + dados_sensor_global.offset_press;
    double altitude_calculada = 44330.0 * (1.0 - pow(press_converted / SEA_LEVEL_PRESSURE, 0.1903));
    dados_sensor_global.altitude = altitude_calculada + dados_sensor_global.offset_alt; 

    // Leitura do Sensor AHT20 
    AHT20_Data aht_data;
    if (aht20_read(I2C_PORT, &aht_data)) {
        dados_sensor_global.umidade_aht = aht_data.humidity + dados_sensor_global.offset_umid; // Aplica offset de umidade
    }

    // Desloca todos os valores que já estão no vetor uma posição à esqueda
    for (int i = 0; i < 19; i++) {
        dados_sensor_global.hist_temp[i] = dados_sensor_global.hist_temp[i + 1];
        dados_sensor_global.hist_umid[i] = dados_sensor_global.hist_umid[i + 1];
        dados_sensor_global.hist_press[i] = dados_sensor_global.hist_press[i + 1];
        dados_sensor_global.hist_alt[i] = dados_sensor_global.hist_alt[i + 1];
    }

    // Adiciona os novos valores na última posição de cada vetor
    dados_sensor_global.hist_temp[19] = dados_sensor_global.temperatura_bmp;
    dados_sensor_global.hist_umid[19] = dados_sensor_global.umidade_aht;
    dados_sensor_global.hist_press[19] = dados_sensor_global.pressao_hpa;
    dados_sensor_global.hist_alt[19] = dados_sensor_global.altitude;
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
    memset(&dados_sensor_global, 0, sizeof(SENSOR_DATA));

    // Definição de limites mínimos e máximos padrão
    dados_sensor_global.limite_min_temp = 20; 
    dados_sensor_global.limite_max_temp = 40; 
    dados_sensor_global.limite_min_umid = 30;
    dados_sensor_global.limite_max_umid = 65;
    dados_sensor_global.limite_min_press = 950;
    dados_sensor_global.limite_max_press = 1200;
    dados_sensor_global.limite_min_alt = 0;
    dados_sensor_global.limite_max_alt = 1500;

}

SENSOR_DATA* get_sensor_data(void) {
    return &dados_sensor_global;
}

void setar_temp_offset(float offset) {
    dados_sensor_global.offset_temp = offset;
}

void setar_press_offset(float offset) {
    dados_sensor_global.offset_press = offset;
}

void setar_umid_offset(float offset) {
    dados_sensor_global.offset_umid = offset;
}

void setar_alt_offset(float offset) {
    dados_sensor_global.offset_alt = offset;
}

void setar_limites_temp(int min, int max) {
    dados_sensor_global.limite_min_temp = min;
    dados_sensor_global.limite_max_temp = max;
}

void setar_limites_umid(int min, int max) {
    dados_sensor_global.limite_min_umid = min;
    dados_sensor_global.limite_max_umid = max;
}

void setar_limites_press(int min, int max) {
    dados_sensor_global.limite_min_press = min;
    dados_sensor_global.limite_max_press = max;
}

void setar_limites_alt(int min, int max) {
    dados_sensor_global.limite_min_alt = min;
    dados_sensor_global.limite_max_alt = max;
}