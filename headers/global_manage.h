#ifndef GLOBAL_MANAGE_H
#define GLOBAL_MANAGE_H

#include "pico/stdlib.h"

// Estrutura completa para armazenar todos os dados e configurações do sistema.
typedef struct {
    
    // Dados lidos dos sensores
    float temperatura_bmp;
    float umidade_aht;
    float pressao_hpa;
    float altitude;

    // Offsets de calibração configurados pelo usuário na web
    float offset_temp;
    float offset_press;
    float offset_umid; 
    float offset_alt;

    // Limites dos alertas para todas as grandezas medidas
    int limite_min_temp;
    int limite_max_temp;
    int limite_min_umid;
    int limite_max_umid;
    int limite_min_press;
    int limite_max_press;
    int limite_min_alt;
    int limite_max_alt;
    
    // Dados para o gráfico da interface web (últimos 20 valores)
    float hist_temp[20];
    float hist_umid[20];
    float hist_press[20];
    float hist_alt[20];

} SENSOR_DATA;


// Inicializa o I2C, os sensores e também a estrutura de dados 
void setup_sensor_manager(void);

// Função para retonar um ponteiro do tipo da estrutura de dados, utilizado no webserver para comunicação das variáveis
SENSOR_DATA* get_sensor_data(void);

// Lê os valores dos sensores
void ler_sensores();

// Funções abaixo definem o offset do sensor tanto quanto os limites mínimo e máximo das leituras realizadas
void setar_temp_offset(float offset);

void setar_press_offset(float offset);

void setar_umid_offset(float offset);

void setar_alt_offset(float offset);

void setar_limites_temp(int min, int max);

void setar_limites_umid(int min, int max);

void setar_limites_press(int min, int max);

void setar_limites_alt(int min, int max);

#endif