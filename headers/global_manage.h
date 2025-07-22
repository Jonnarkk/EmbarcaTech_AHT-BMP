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

    // Offsets de calibração configurados via web
    float offset_temp;
    float offset_press;
    float offset_umid; 
    float offset_alt;

    // [EXPANDIDO] Limites de alerta para todas as propriedades
    int limite_min_temp;
    int limite_max_temp;
    int limite_min_umid;
    int limite_max_umid;
    int limite_min_press;
    int limite_max_press;
    int limite_min_alt;
    int limite_max_alt;
    
    // Dados históricos para o gráfico da interface web (últimos 20 valores)
    float hist_temp[20];
    float hist_umid[20];
    float hist_press[20];
    float hist_alt[20];

} SENSOR_DATA;


/**
 * @brief Inicializa o hardware (I2C, sensores, pinos de alerta) e o timer para leitura periódica.
 * Deve ser chamada uma vez no início do programa.
 */
void setup_sensor_manager(void);

/**
 * @brief Retorna um ponteiro para a estrutura de dados global.
 * Usado pelo servidor web para obter os dados atuais e enviá-los como JSON.
 * @return SENSOR_DATA* Ponteiro para os dados dos sensores.
 */
SENSOR_DATA* get_sensor_data(void);

void ler_sensores();

/**
 * @brief Define o offset de calibração para o sensor de temperatura.
 * @param offset Valor do offset recebido da interface web.
 */
void setar_temp_offset(float offset);

/**
 * @brief Define o offset de calibração para o sensor de pressão.
 * @param offset Valor do offset recebido da interface web.
 */
void setar_press_offset(float offset);

void setar_umid_offset(float offset);
void setar_alt_offset(float offset);

/**
 * @brief Define os limites mínimo e máximo de temperatura para os alertas.
 * @param min Limite mínimo recebido da interface web.
 * @param max Limite máximo recebido da interface web.
 */
void setar_limites_temp(int min, int max);

void setar_limites_umid(int min, int max);
void setar_limites_press(int min, int max);
void setar_limites_alt(int min, int max);

#endif