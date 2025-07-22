#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"      // Bibloteca de PWM para controle de sinais sonoros
#include "hardware/clocks.h"   // Biblioteca de clocks do RP2040 para gerenciar frequências

// Cabeçalhos criados 
#include "server.h"
#include "global_manage.h"
#include "connect_wifi.h"
#include "init_config.h"
#include "ssd1306.h"
#include "matriz.h"
#include "pio_matrix.pio.h"

// Inclusão dos cabeçalhos do FreeRTOS
#include "FreeRTOS.h"          // Kernel FreeRTOS
#include "task.h"              // API de criação e controle de tarefas FreeRTOS

// Definição do Botão para iniciar programa
#define BOTAO_A 5

// --- Variáveis Globais ---
bool alerta = false;       // Variável que indica se um alerta está ativo
bool is_connected = false;    // Variável que indica se o sistema foi ativado pelo botão
ssd1306_t ssd;                  // Variável da estrutura do display OLED

// Função para lidar com a interrupção do botão
void gpio_callback(uint gpio, uint32_t events) {
    // Tratamento de debounce
    static uint32_t last_press_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_press_time < 250) {
        return; 
    }
    last_press_time = current_time;

    // Ação principal da interrupção: ativa o sistema
    if (gpio == BOTAO_A) {
        is_connected = true; // Ativa a flag 'is_connected' para iniciar o monitoramento de alertas
        printf("Botão pressionado! Variável 'is_connected' ligada.\n");
    }
}

// Função para configurar o botão A
void setup_button() {
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A); 

    
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Função para iniciar a conexão do Wi-Fi e servidor na Web
void vServerTask()
{
    // Exibe mensagem no display
    ssd1306_fill(&ssd, 0);
    ssd1306_draw_string(&ssd, "ESTABELECENDO", centralizar_texto("ESTABELECENDO"), 30);
    ssd1306_draw_string(&ssd, "CONEXAO...", centralizar_texto("CONEXAO..."), 40);
    ssd1306_send_data(&ssd);
    vTaskDelay(pdMS_TO_TICKS(2000));     

    // Chama a função para conectar ao Wi-Fi e obter o endereço IP
    char* result = connect_wifi();
    // Inicia o servidor HTTP para a interface web
    start_http_server();

    // Exibe informações sobre o estado de conexão do servidor
    ssd1306_fill(&ssd, 0);
    ssd1306_draw_string(&ssd, "CONECTADO", centralizar_texto("CONECTADO"), 0);
    ssd1306_draw_string(&ssd, "IP:", 0, 15);
    if (result) {
        ssd1306_draw_string(&ssd, result, 25, 15);
    }
    ssd1306_draw_string(&ssd, "Iniciar", 0, 35);
    ssd1306_draw_string(&ssd, "-> Botao A", 0, 45);
    ssd1306_send_data(&ssd);

    // Loop da tarefa
    while (true)
    {
        cyw43_arch_poll();
        // Libera o processador para outras tarefas por 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    cyw43_arch_deinit();
}

// Função exclusiva para a leitura do sensor
void vSensorTask()
{
    // Inicializa o gerenciador de sensores (I2C, etc.)
    setup_sensor_manager();

    // Loop principal da tarefa
    while (true)
    {
        // Chama a função que lê os dados de todos os sensores
        ler_sensores();
        // Libera o processador para outras tarefas por 2 segundos e também faz com que a leitura dos dados do sensor ocorra de 2 em 2 segundos
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Função de Alerta da matriz e ativacação da flag de alerta
void vAlertaMatrizTask()
{
    // Configura o PIO para controlar a matriz de LEDs
    PIO pio = pio0;
    uint sm = pio_init(pio); 

    // Obtém o ponteiro para a estrutura de dados dos sensores
    SENSOR_DATA *data = get_sensor_data();

    // Loop principal da tarefa
    while (true)
    {
        // Só executa a lógica se o Botão A foi pressionado
        if (is_connected) 
        {
            // Verifica se algum valor ultrapassou o limite MÁXIMO
            if (data->temperatura_bmp > data->limite_max_temp || data->pressao_hpa > data->limite_max_press || data->umidade_aht > data->limite_max_umid || data->altitude > data->limite_max_alt) 
            {
                desenhar_alerta(pio, sm); // Desenha padrão de alerta na matriz
                desenha_display_alerta_sup(&ssd, data);  // Mostra alerta no OLED
                alerta = true; // Ativa a flag global de alerta
            }
            // Verifica se algum valor ficou abaixo do limite MÍNIMO
            else if (data->temperatura_bmp < data->limite_min_temp || data->pressao_hpa < data->limite_min_press || data->umidade_aht < data->limite_min_umid || data->altitude < data->limite_min_alt) 
            {
                desenhar_alerta(pio, sm); // Desenha padrão de alerta na matriz
                desenha_display_alerta_inf(&ssd, data);   // Mostra alerta no OLED
                alerta = true; // Ativa a flag global de alerta
            }
            // Se não há alertas
            else 
            {
                apagar_matriz(pio, sm);                 // Apaga a matriz de LEDs
                desenha_display_normal(&ssd, data);     // Mostra dados normais no OLED
                alerta = false; // Desativa a flag global de alerta
            }
        }
        // Para não consumir demais o processador
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Função de Alerta do LED e do Buzzer
void vAlertaLEDTask(void *pvParameters) {
    // Inicializa o PWM para o buzzer e os GPIOs para os LEDs
    config_buzzer();
    inicializar_leds();
    
    while (1) {
        // Só executa se a flag de alerta e a de conexão estiverem ativas
        if (alerta && is_connected) {  
            // Em modo alerta, alterna LEDs e gera tom no buzzer
            gpio_put(LED_RED, 1);
            gpio_put(LED_GREEN, 1);
            pwm_set_gpio_level(BUZZER_PIN, 2048); // Liga o som
            vTaskDelay(pdMS_TO_TICKS(200));

            gpio_put(LED_GREEN, 0);
            gpio_put(LED_RED, 1);
            pwm_set_gpio_level(BUZZER_PIN, 0); // Desliga o som
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_put(LED_RED, 0);
        } else {
            // Se não há alerta, garante que os atuadores - LED e Buzzer - estejam desligados
            gpio_put(LED_GREEN, 0);
            gpio_put(LED_RED, 0);
            pwm_set_gpio_level(BUZZER_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// Função principal
int main()
{
    // Inicializa a comunicação serial e o display
    stdio_init_all();
    display_init(&ssd);

    setup_button();

    // Cria as tarefas do FreeRTOS
    xTaskCreate(vServerTask, "Server Task", 2048, NULL, 1, NULL);
    xTaskCreate(vSensorTask, "Sensor Task", 1024, NULL, 1, NULL);
    xTaskCreate(vAlertaMatrizTask, "Matriz Task", 1024, NULL, 1, NULL);
    xTaskCreate(vAlertaLEDTask, "LED Task", 1024, NULL, 1, NULL);

    
    vTaskStartScheduler();

    // O código nunca deve chegar aqui. Se chegar, algo deu muito errado.
    while(1) {};
}