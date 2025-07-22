// Ficheiro: server.c (Versão Final com Todos os Endpoints e Requisitos)

#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdlib.h>
#include "global_manage.h"

// =================================================================================
// HTML / CSS / JavaScript - FINAL COM TODOS OS CAMPOS E GRÁFICOS
// =================================================================================
const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><title>Estação Meteorológica</title><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<style>"
    "body{font-family:sans-serif;background:#f0f2f5;display:flex;justify-content:center;padding:10px;margin:0}"
    ".container{width:100%;max-width:800px;background:white;padding:20px;border-radius:10px;box-shadow:0 4px 12px rgba(0,0,0,0.1)}"
    "h1,h2{text-align:center;color:#333}hr{border:1px solid #eee;margin:20px 0}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:20px;text-align:center}"
    ".card{background:#f8f9fa;padding:15px;border-radius:8px;border:1px solid #ddd}"
    ".card p{margin:0;font-size:1.5rem;font-weight:bold;color:#007bff}.card span{font-size:0.9rem;color:#6c757d}"
    ".charts-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-top:20px}"
    ".chart-container{width:100%}"
    ".form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-top:20px}"
    ".form-group{display:flex;flex-direction:column}label{margin-bottom:5px;font-weight:bold;color:#555;text-align:left}"
    "input{padding:8px;border:1px solid #ccc;border-radius:5px;font-size:1rem;width:calc(100% - 18px)}"
    "button{padding:10px;border:none;background:#28a745;color:white;border-radius:5px;cursor:pointer;margin-top:10px}"
    ".limit-inputs{display:flex;gap:10px} .limit-inputs input{width:calc(50% - 28px)}"
    "</style>"
    "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
    "<script>"
    "let tempChart,umidChart,pressChart,altChart;"
    "function createChart(elId,label,color){const ctx=document.getElementById(elId).getContext('2d');"
    "return new Chart(ctx,{type:'line',data:{labels:[],datasets:[{label:label,data:[],borderColor:color,backgroundColor:color+'80',tension:0.1}]},"
    "options:{scales:{y:{beginAtZero:false}},animation:false,plugins:{legend:{display:true}}}})}"
    "function initCharts(){tempChart=createChart('tempChart','Temperatura (°C)','rgb(255,99,132)');"
    "umidChart=createChart('umidChart','Umidade (%)','rgb(54,162,235)');"
    "pressChart=createChart('pressChart','Pressão (hPa)','rgb(75,192,192)');"
    "altChart=createChart('altChart','Altitude (m)','rgb(153,102,255)');}"
    "function setConfig(param){const v=document.getElementById('input_'+param).value;fetch('/config?'+param+'='+v).then(()=>document.getElementById('input_'+param).blur())}"
    "function setLimits(param){const min=document.getElementById('input_limite_min_'+param).value;"
    "const max=document.getElementById('input_limite_max_'+param).value;"
    "fetch('/config?limite_min_'+param+'='+min+'&limite_max_'+param+'='+max).then(()=>{document.getElementById('input_limite_min_'+param).blur();document.getElementById('input_limite_max_'+param).blur()})}"
    "function atualizarDados(){fetch('/dados_sensores').then(r=>r.json()).then(d=>{"
    "document.getElementById('temp').innerText=d.temp.toFixed(2);document.getElementById('umid').innerText=d.umid.toFixed(2);"
    "document.getElementById('press').innerText=d.press.toFixed(2);document.getElementById('alt').innerText=d.alt.toFixed(2);"
    "if(!document.activeElement.id.includes('input')){document.getElementById('input_offset_temp').value=d.offset_temp;"
    "document.getElementById('input_offset_press').value=d.offset_press;"
    "document.getElementById('input_offset_umid').value=d.offset_umid;"
    "document.getElementById('input_offset_alt').value=d.offset_alt;"
    "document.getElementById('input_limite_min_temp').value=d.limite_min_temp;document.getElementById('input_limite_max_temp').value=d.limite_max_temp;"
    "document.getElementById('input_limite_min_umid').value=d.limite_min_umid;document.getElementById('input_limite_max_umid').value=d.limite_max_umid;"
    "document.getElementById('input_limite_min_press').value=d.limite_min_press;document.getElementById('input_limite_max_press').value=d.limite_max_press;"
    "document.getElementById('input_limite_min_alt').value=d.limite_min_alt;document.getElementById('input_limite_max_alt').value=d.limite_max_alt;}"
    "tempChart.data.datasets[0].data=d.hist_temp;umidChart.data.datasets[0].data=d.hist_umid;"
    "pressChart.data.datasets[0].data=d.hist_press;altChart.data.datasets[0].data=d.hist_alt;"
    "const labels=d.hist_labels;tempChart.data.labels=labels;umidChart.data.labels=labels;pressChart.data.labels=labels;altChart.data.labels=labels;"
    "tempChart.update();umidChart.update();pressChart.update();altChart.update();"
    "}).catch(e=>console.error('Erro:',e))}"
    "window.onload=()=>{initCharts();atualizarDados();setInterval(atualizarDados,2000)};"
    "</script></head><body>"
    "<div class=container><h1>Estação Meteorológica</h1><div class=grid>"
    "<div class=card><p id=temp>--</p><span>Temperatura (°C)</span></div><div class=card><p id=umid>--</p><span>Umidade (%)</span></div>"
    "<div class=card><p id=press>--</p><span>Pressão (hPa)</span></div><div class=card><p id=alt>--</p><span>Altitude (m)</span></div></div>"
    "<div class=charts-grid><div class=chart-container><canvas id=tempChart></canvas></div><div class=chart-container><canvas id=umidChart></canvas></div>"
    "<div class=chart-container><canvas id=pressChart></canvas></div><div class=chart-container><canvas id=altChart></canvas></div></div><hr>"
    "<h2>Configurações de Calibração</h2><div class=form-grid>"
    "<div class=form-group><label>Offset Temperatura:</label><input type=number step=0.1 id=input_offset_temp><button onclick=\"setConfig('offset_temp')\">Definir</button></div>"
    "<div class=form-group><label>Offset Umidade:</label><input type=number step=0.1 id=input_offset_umid><button onclick=\"setConfig('offset_umid')\">Definir</button></div>"
    "<div class=form-group><label>Offset Pressão:</label><input type=number step=0.1 id=input_offset_press><button onclick=\"setConfig('offset_press')\">Definir</button></div>"
    "<div class=form-group><label>Offset Altitude:</label><input type=number step=0.1 id=input_offset_alt><button onclick=\"setConfig('offset_alt')\">Definir</button></div></div>"
    "<h2>Configurações de Alertas</h2><div class=form-grid>"
    "<div class=form-group><label>Limites Temperatura (°C):</label><div class=limit-inputs><input type=number placeholder=Min id=input_limite_min_temp> <input type=number placeholder=Max id=input_limite_max_temp></div><button onclick=\"setLimits('temp')\">Definir</button></div>"
    "<div class=form-group><label>Limites Umidade (%):</label><div class=limit-inputs><input type=number placeholder=Min id=input_limite_min_umid> <input type=number placeholder=Max id=input_limite_max_umid></div><button onclick=\"setLimits('umid')\">Definir</button></div>"
    "<div class=form-group><label>Limites Pressão (hPa):</label><div class=limit-inputs><input type=number placeholder=Min id=input_limite_min_press> <input type=number placeholder=Max id=input_limite_max_press></div><button onclick=\"setLimits('press')\">Definir</button></div>"
    "<div class=form-group><label>Limites Altitude (m):</label><div class=limit-inputs><input type=number placeholder=Min id=input_limite_min_alt> <input type=number placeholder=Max id=input_limite_max_alt></div><button onclick=\"setLimits('alt')\">Definir</button></div>"
    "</div></div></body></html>";




typedef enum { SENDING_HEADERS, SENDING_BODY } SENDING_PHASE;
typedef struct HTTP_STATE_T {
    char response_buffer[2048];
    const char *response_ptr;  
    size_t response_len;       
    SENDING_PHASE phase;       
} HTTP_STATE;

static void http_close_and_free(struct tcp_pcb *tpcb, HTTP_STATE *state) {
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    if (state) {
        free(state);
    }
    tcp_close(tpcb);
}

static void http_err(void *arg, err_t err) {
    if (arg) {
        free(arg);
    }
}

static err_t http_send_data(struct tcp_pcb *tpcb, HTTP_STATE *state) {
    u16_t available_len = tcp_sndbuf(tpcb);
    if (available_len > TCP_MSS) {
        available_len = TCP_MSS;
    }
    u16_t send_len = state->response_len < available_len ? state->response_len : available_len;
    if (send_len == 0) return ERR_OK;

    err_t err = tcp_write(tpcb, state->response_ptr, send_len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        http_close_and_free(tpcb, state);
        return err;
    }
    state->response_ptr += send_len;
    state->response_len -= send_len;
    return ERR_OK;
}

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    HTTP_STATE *state = (HTTP_STATE *)arg;
    if (state->response_len > 0) {
        http_send_data(tpcb, state);
    } else {
        if (state->phase == SENDING_HEADERS) {
            state->phase = SENDING_BODY;
            state->response_ptr = HTML_BODY;
            state->response_len = strlen(HTML_BODY);
            http_send_data(tpcb, state);
        } else {
            http_close_and_free(tpcb, state);
        }
    }
    return ERR_OK;
}

static void build_hist_string(char* dest, size_t dest_size, float* source) {
    int offset = 0;
    offset += snprintf(dest + offset, dest_size - offset, "[");
    for(int i=0; i<20; ++i) {
        offset += snprintf(dest + offset, dest_size - offset, "%.2f%s", source[i], (i<19) ? "," : "");
    }
    snprintf(dest + offset, dest_size - offset, "]");
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        http_close_and_free(tpcb, (HTTP_STATE *)arg);
        return ERR_OK;
    }
    tcp_recved(tpcb, p->tot_len);
    
    char req_buffer[256];
    pbuf_copy_partial(p, req_buffer, sizeof(req_buffer) - 1, 0);
    req_buffer[sizeof(req_buffer) - 1] = '\0';
    pbuf_free(p);

    if (arg) { 
        return ERR_OK;
    }
    
    HTTP_STATE *state = malloc(sizeof(HTTP_STATE));
    tcp_arg(tpcb, state);

    if (strncmp(req_buffer, "GET / ", 6) == 0) {
        state->phase = SENDING_HEADERS;
        state->response_len = snprintf(state->response_buffer, sizeof(state->response_buffer),
            "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\nContent-Length: %d\r\n\r\n", strlen(HTML_BODY));
        state->response_ptr = state->response_buffer;

    } else if (strncmp(req_buffer, "GET /dados_sensores", 19) == 0) {
        state->phase = SENDING_BODY;
        SENSOR_DATA* data = get_sensor_data();
        char json_body[2048]; 
        char hist_temp_str[300], hist_umid_str[300], hist_press_str[300], hist_alt_str[300];
        
        build_hist_string(hist_temp_str, sizeof(hist_temp_str), data->hist_temp);
        build_hist_string(hist_umid_str, sizeof(hist_umid_str), data->hist_umid);
        build_hist_string(hist_press_str, sizeof(hist_press_str), data->hist_press);
        build_hist_string(hist_alt_str, sizeof(hist_alt_str), data->hist_alt);
        
        snprintf(json_body, sizeof(json_body),
            "{\"temp\":%.2f,\"umid\":%.2f,\"press\":%.2f,\"alt\":%.2f,"
            "\"offset_temp\":%.2f,\"offset_press\":%.2f,\"offset_umid\":%.2f,\"offset_alt\":%.2f,"
            "\"limite_min_temp\":%d,\"limite_max_temp\":%d,"
            "\"limite_min_umid\":%d,\"limite_max_umid\":%d,"
            "\"limite_min_press\":%d,\"limite_max_press\":%d,"
            "\"limite_min_alt\":%d,\"limite_max_alt\":%d,"
            "\"hist_labels\":[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20],"
            "\"hist_temp\":%s,\"hist_umid\":%s,\"hist_press\":%s,\"hist_alt\":%s}",
            data->temperatura_bmp, data->umidade_aht, data->pressao_hpa, data->altitude,
            data->offset_temp, data->offset_press, data->offset_umid, data->offset_alt,
            data->limite_min_temp, data->limite_max_temp, data->limite_min_umid, data->limite_max_umid,
            data->limite_min_press, data->limite_max_press, data->limite_min_alt, data->limite_max_alt,
            hist_temp_str, hist_umid_str, hist_press_str, hist_alt_str);
        
        state->response_len = snprintf(state->response_buffer, sizeof(state->response_buffer),
            "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s",
            strlen(json_body), json_body);
        state->response_ptr = state->response_buffer;

    } else if (strncmp(req_buffer, "GET /config?", 12) == 0) {
        state->phase = SENDING_BODY;
        char* value_ptr;
        
        if ((value_ptr = strstr(req_buffer, "offset_temp="))) { setar_temp_offset(atof(value_ptr + 12)); }
        else if ((value_ptr = strstr(req_buffer, "offset_umid="))) { setar_umid_offset(atof(value_ptr + 12)); }
        else if ((value_ptr = strstr(req_buffer, "offset_press="))) { setar_press_offset(atof(value_ptr + 13)); }
        else if ((value_ptr = strstr(req_buffer, "offset_alt="))) { setar_alt_offset(atof(value_ptr + 11)); }
        else { 
            char* min_str, *max_str;
            if ((min_str = strstr(req_buffer, "limite_min_temp=")) && (max_str = strstr(req_buffer, "limite_max_temp="))) {
                setar_limites_temp(atoi(min_str + 16), atoi(max_str + 16));
            } else if ((min_str = strstr(req_buffer, "limite_min_umid=")) && (max_str = strstr(req_buffer, "limite_max_umid="))) {
                setar_limites_umid(atoi(min_str + 16), atoi(max_str + 16));
            } else if ((min_str = strstr(req_buffer, "limite_min_press=")) && (max_str = strstr(req_buffer, "limite_max_press="))) {
                setar_limites_press(atoi(min_str + 17), atoi(max_str + 17));
            } else if ((min_str = strstr(req_buffer, "limite_min_alt=")) && (max_str = strstr(req_buffer, "limite_max_alt="))) {
                setar_limites_alt(atoi(min_str + 15), atoi(max_str + 15));
            }
        }

        const char* msg = "OK";
        state->response_len = snprintf(state->response_buffer, sizeof(state->response_buffer),
            "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/plain\r\nContent-Length: %d\r\n\r\n%s", strlen(msg), msg);
        state->response_ptr = state->response_buffer;
        
    } else {
        state->phase = SENDING_BODY;
        const char* msg = "<h1>404 Not Found</h1>";
        state->response_len = snprintf(state->response_buffer, sizeof(state->response_buffer),
            "HTTP/1.1 404 Not Found\r\nConnection: close\r\nContent-Type: text/html\r\nContent-Length: %d\r\n\r\n%s", strlen(msg), msg);
        state->response_ptr = state->response_buffer;
    }
    
    tcp_sent(tpcb, http_sent);
    http_send_data(tpcb, state);
    tcp_output(tpcb);

    return ERR_OK;
}

static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, http_recv);
    tcp_err(newpcb, http_err);
    return ERR_OK;
}

void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, http_accept);
    printf("Servidor final e completo rodando na porta 80.\n");
}