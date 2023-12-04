/* HTTP2 Web Radio Listener Device

   Part of WCWebCamServer, LiteSound projects

   Copyright (c) 2023 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "defs.h"

#include "wcstrutils.h"

#include "wcprotocol.h"

#include "ble_config.h"

#include "esp_wifi.h"
#include <sys/time.h>
#include "lwip/apps/sntp.h"
#include "driver/gpio.h"
#include "http2_protoclient.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"

#include "OWIcrc.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"

#include "math.h"

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#else
#define ESP_IDF_VERSION_VAL(major, minor, patch) 1
#endif

const char *WC_TAG = "relayhttp2-rsp";

/* mac address for device */
static char mac_str[13];
static cJSON * device_meta_data = NULL;
static char device_name[32];

/* wifi config */
#define APP_WIFI_SSID CONFIG_WIFI_SSID
#define APP_WIFI_PASS CONFIG_WIFI_PASSWORD

/* HTTP2 HOST config */
// The HTTP/2 server to connect to
#define HTTP2_SERVER_URI   CONFIG_SERVER_URI
// The user's name
#define HTTP2_SERVER_NAME  CONFIG_SERVER_NAME
// The user's password
#define HTTP2_SERVER_PASS  CONFIG_SERVER_PASS

//Blink LED modes
#define BLINK_LED_OFF             0x0000
#define BLINK_LED_ON              0xFFFF
#define BLINK_LED_BLE             0x0033
#define BLINK_LED_WIFI_START      0xFFFF
#define BLINK_LED_WIFI_CONNECTED  0xFFF8
#define BLINK_LED_HOST_CONNECTED  0xF0F0
#define BLINK_LED_AUTHORIZED      0x0001
#define BLINK_LED_ERROR           0xAAAA


/* JSON-RPC device metadata */
/* device's write char to identify */
static const char * JSON_BLE_CHAR         =  "ble_char";

/* MSGS */
static const char * JSON_RPC_T            =  "t";
static const char * JSON_RPC_EXT_POW      =  "extp";
static const char * JSON_RPC_HEAT         =  "heat";
static const char * JSON_RPC_MODE         =  "mode";
static const char * JSON_RPC_HYST         =  "hyst";
static const char * JSON_RPC_TARGET_T     =  "trgt";
static const char * JSON_RPC_MODE_ID      =  "id";
static const char * JSON_RPC_MODE_VALUE   =  "value";
static const char * JSON_RPC_MODE_VALUES  =  "values";

static const char * JSON_RPC_START_BROAD  =  "startbrd";
static const char * JSON_RPC_STOP_BROAD   =  "stopbrd";
static const char * JSON_RPC_IS_BROAD     =  "isbrd";
static const char * JSON_RPC_STATUS       =  "status";
static const char * JSON_RPC_GET_T        =  "gett";
static const char * JSON_RPC_MODE_LIST    =  "lstmodes";
static const char * JSON_RPC_GET_EXT_POW  =  "getextp";
static const char * JSON_RPC_SET_HYST     =  "sethyst";
static const char * JSON_RPC_GET_HYST     =  "gethyst";
static const char * JSON_RPC_SET_MODE     =  "setmode";
static const char * JSON_RPC_GET_MODE     =  "getmode";
static const char * JSON_RPC_CUR_MODE     =  "curmode";
static const char * JSON_RPC_RUN_MODE     =  "runmode";

/* Modes in state-machina */
const int WIFI_CONNECTED_BIT = BIT0;
const int HOST_CONNECTED_BIT = BIT1;
const int AUTHORIZED_BIT     = BIT2;
const int MODE_SETIME        = BIT3;
// autorization step. is device need to authorize
const int  MODE_AUTH         = BIT4;
const int  MODE_MEASURE_T    = BIT5;
const int  MODE_SEND_T       = BIT6;
const int  MODE_HAS_T        = BIT7;
// recieve/send msg. is need to send msg from pool to server
const int  MODE_RECIEVE_MSG  = BIT8;
const int  MODE_SEND_MSG     = BIT9;
const int  MODE_T_VARS_RW    = BIT10;
const int  MODE_SEND_BROAD   = BIT11;

const int  MODE_ALL          = 0xfffffe;

/* global vars */
volatile int  wifi_connect_errors = 0;           // wifi connection failed tryes count
volatile int  connect_errors = 0;                // connection failed tryes count
volatile uint16_t  CUR_BLINK_PATTERN_LOC = 1;    // current LED blink pattern pos
/* T-variables (protected for multi-threaded purposes) */
#define MAX_T_MODES 3
static float   T_MODES [MAX_T_MODES] = { 5.0, 20.0, 23.0 };
static uint8_t T_MODE_CUR = 0;
static float   TEMPERATURE = 0.0;
static float   T_HYST = 1.0;
static bool    EXT_POWER = false;
static bool    HEATING = false;
static bool    MEASURED_EXT_POWER = false;
static bool    SUPPLYED_HEATING = false;
static bool    T_VARS_CHANGED = false;
volatile uint16_t CUR_BLINK_PATTERN = 0;        // current LED blink pattern
// broadcasting
static bool       ENABLE_BROADCAST = false;
volatile uint32_t BROADCAST_TIME = 0;
volatile bool     LST_BRD_STATUS_EXT_POW = false;
volatile bool     LST_BRD_STATUS_HEATING = false;
volatile float    LST_BRD_STATUS_T = 0.0;
volatile float    LST_BRD_STATUS_TARGET_T = 5.0;

/* state routes */
static EventGroupHandle_t client_state = NULL; // state mask locker for multi-thread access
static SemaphoreHandle_t vars_mux = NULL;      // mt vars set/get

#define LOCK_MUX   (xSemaphoreTakeRecursive(vars_mux, portMAX_DELAY) == pdTRUE)
#define UNLOCK_MUX xSemaphoreGiveRecursive(vars_mux)
#define PROTECTED_GET_MUX(x, y, z) \
            if (LOCK_MUX) { \
                x = y; \
                UNLOCK_MUX; \
            } else \
                x = z;
#define PROTECTED_SET_MUX(x, y) \
            if (LOCK_MUX) { \
                x = y; \
                UNLOCK_MUX; \
            }

uint16_t locked_get_blink_pattern() {
    float val;
    PROTECTED_GET_MUX(val, CUR_BLINK_PATTERN, 0.0);
    return val;
}

void locked_set_blink_pattern(uint16_t value) {
    PROTECTED_SET_MUX(CUR_BLINK_PATTERN, value);
}

bool locked_get_is_broadcast() {
    bool val;
    PROTECTED_GET_MUX(val, ENABLE_BROADCAST, false);
    return val;
}

void locked_set_is_broadcast(bool value) {
    if (LOCK_MUX) {
        if (ENABLE_BROADCAST != value) {
            ENABLE_BROADCAST = value;
            T_VARS_CHANGED = true;
        }
        UNLOCK_MUX;
    }
}

uint32_t locked_get_broadcast_time() {
    uint32_t val;
    PROTECTED_GET_MUX(val, BROADCAST_TIME, 0);
    return val;
}

void locked_set_broadcast_time(uint32_t value) {
    PROTECTED_SET_MUX(BROADCAST_TIME, value);
}


float locked_get_t_mode(uint8_t mode) {
    float val;
    PROTECTED_GET_MUX(val, T_MODES[mode], 0.0);
    return val;
}

void locked_set_t_mode(uint8_t mode, float value) {
    if (LOCK_MUX) {
        if (value < 0.0) value = 0.0;
        if (value > 50.0) value = 50.0;
        if (T_MODES[mode] != value) {
            T_MODES[mode] = value;
            T_VARS_CHANGED = true;
        }
        UNLOCK_MUX;
    }
}

uint8_t locked_get_cur_t_mode_id() {
    uint8_t val;
    PROTECTED_GET_MUX(val, T_MODE_CUR, 0);
    return val;
}

float locked_get_cur_t_mode_value() {
    float val;
    PROTECTED_GET_MUX(val, T_MODES[T_MODE_CUR], 0.0);
    return val;
}

void locked_set_cur_t_mode_id(uint8_t value) {
    if (LOCK_MUX) {
        if (T_MODE_CUR != value) {
            T_MODE_CUR = value;
            T_VARS_CHANGED = true;
        }
        UNLOCK_MUX;
    }
}

float locked_get_temperature() {
    float val;
    PROTECTED_GET_MUX(val, TEMPERATURE, 0.0);
    return val;
}

void locked_set_temperature(float value) {
    PROTECTED_SET_MUX(TEMPERATURE, value);
}

float locked_get_hyst() {
    float val;
    PROTECTED_GET_MUX(val, T_HYST, 1.0);
    return val;
}

void locked_set_hyst(float value) {
    if (LOCK_MUX) {
        if (value < 0.125) value = 0.125;
        if (value > 2.5) value = 2.5;
        if (T_HYST != value) {
            T_HYST = value;
            T_VARS_CHANGED = true;
        }
        UNLOCK_MUX;
    }
}

bool locked_get_ext_power() {
    bool val;
    PROTECTED_GET_MUX(val, EXT_POWER, 0.0);
    return val;
}

void locked_set_ext_power(bool value) {
    PROTECTED_SET_MUX(MEASURED_EXT_POWER, value);
}

bool locked_syn_ext_power() {
    bool res;
    if (LOCK_MUX) {
        res = (EXT_POWER != MEASURED_EXT_POWER);
        EXT_POWER = MEASURED_EXT_POWER;
        UNLOCK_MUX;
    } else res = false;
    return res;
}

bool locked_get_heating() {
    bool val;
    PROTECTED_GET_MUX(val, HEATING, 0.0);
    return val;
}

void locked_set_heating(bool value) {
    PROTECTED_SET_MUX(SUPPLYED_HEATING, value);
}

bool locked_syn_heating() {
    bool res;
    if (LOCK_MUX) {
        res = (HEATING != SUPPLYED_HEATING);
        HEATING = SUPPLYED_HEATING;
        UNLOCK_MUX;
    } else res = false;
    return res;
}

/* thread-safe get states route */
static uint32_t locked_GET_STATES() {
    return xEventGroupGetBits(client_state);
}

/* thread-safe check state route */
static bool locked_CHK_STATE(uint32_t astate) {
    bool val = locked_GET_STATES(client_state) & astate;
    return val;
}

static void locked_CLR_STATE(uint32_t astate) {
    xEventGroupClearBits(client_state, astate);
}

static void locked_SET_STATE(uint32_t astate) {
    xEventGroupSetBits(client_state, astate);
}

/* thread-safe clear all states route */
static void locked_CLR_ALL_STATES() {
    locked_CLR_STATE(MODE_ALL);
}

/* timers */
static esp_timer_handle_t msgs_recieve;
static esp_timer_handle_t msgs_send;
static esp_timer_handle_t meas_temp;
static esp_timer_handle_t broadcast;
static esp_timer_handle_t blink_led;
#define SEND_MSG_TIMER_DELTA                   1000000
#define SEND_MSG_NO_EP_TIMER_DELTA             2000000
#define GET_MSG_TIMER_DELTA                    4000000
#define GET_MSG_NO_EP_TIMER_DELTA              8000000
#define BROADCAST_TIMER_DELTA                  15000000
#define MEASURE_TEMPERATURE_TIMER_DELTA        10000000
#define MEASURE_TEMPERATURE_NO_EP_TIMER_DELTA  30000000
#define LED_BLINK_TIMER_DELTA                  150000

/* forward decrlarations */
void finalize_app();

static void set_time(void)
{
    struct timeval tv = {
        .tv_sec = 1509449941,
    };
    struct timezone tz;
    memset(&tz, 0, sizeof(tz));
    settimeofday(&tv, &tz);

    /* Start SNTP service */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();
}

static void consume_protocol_error() {
    if (h2pc_get_last_error() == REST_ERR_NO_SUCH_SESSION) {
        locked_CLR_ALL_STATES();
        locked_SET_STATE(MODE_AUTH);
    }
}

/* disconnect from host. reset all states */
static void disconnect_host() {
    if (locked_CHK_STATE(HOST_CONNECTED_BIT))
        h2pc_disconnect_http2();
    else
        h2pc_reset_buffers();
    locked_CLR_ALL_STATES();
    locked_set_blink_pattern(BLINK_LED_ERROR);
}

/* connect to host */
static void connect_to_http2() {
    disconnect_host();

    char * addr;
    if (WC_CFG_VALUES != NULL)
        addr = get_cfg_value(CFG_HOST_NAME);
    else
        addr = HTTP2_SERVER_URI;

    if (h2pc_connect_to_http2(addr)) {
        connect_errors = 0;
        locked_SET_STATE(HOST_CONNECTED_BIT | MODE_AUTH);
    } else
        connect_errors++;
}

static void send_authorize() {
    ESP_LOGI(WC_TAG, "Trying to authorize");

    const char * _name;
    const char * _pwrd;
    const char * _device;

    if (WC_CFG_VALUES != NULL) {
        _name =  get_cfg_value(CFG_USER_NAME);
        _pwrd =  get_cfg_value(CFG_USER_PASSWORD);
        _device = get_cfg_value(CFG_DEVICE_NAME);
    }
    else {
        _name =  HTTP2_SERVER_NAME;
        _pwrd =  HTTP2_SERVER_PASS;
        _device =  mac_str;
    }

    int res = h2pc_req_authorize_sync(_name, _pwrd, _device, device_meta_data, false);

    if (res == ESP_OK) {
        locked_CLR_STATE(MODE_AUTH);
        locked_SET_STATE(AUTHORIZED_BIT | MODE_RECIEVE_MSG);
        strcpy(device_name, _device);
        ESP_LOGI(WC_TAG, "hash=%s", h2pc_get_sid());
    }
    else
    if (res == H2PC_ERR_PROTOCOL)
        consume_protocol_error();
    else
        disconnect_host();
}

static void recieve_msgs() {
    int res = h2pc_req_get_msgs_sync();
    if (res == ESP_OK)
        locked_CLR_STATE(MODE_RECIEVE_MSG);
}

static void send_msgs() {
    int res = h2pc_req_send_msgs_sync();
    if (res == ESP_OK)
        locked_CLR_STATE(MODE_SEND_MSG);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        locked_set_blink_pattern(BLINK_LED_WIFI_START);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_GOT_IP");
        ESP_LOGI(WC_TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        locked_SET_STATE(WIFI_CONNECTED_BIT);
        locked_SET_STATE(MODE_SETIME);
        wifi_connect_errors = 0;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_DISCONNECTED");

        wifi_connect_errors++;

        sntp_stop();

        if (locked_CHK_STATE(HOST_CONNECTED_BIT)) h2pc_disconnect_http2();
        locked_CLR_ALL_STATES();

        locked_CLR_STATE(WIFI_CONNECTED_BIT);
        h2pc_reset_buffers();
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    locked_set_blink_pattern(BLINK_LED_WIFI_START);
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x00, sizeof(wifi_config_t));

    char * value = get_cfg_value(CFG_SSID_NAME);
    if (value != NULL) {
        strcpy((char *) &(wifi_config.sta.ssid[0]), value);
        ESP_LOGD(WC_TAG, "SSID setted from json config");
    } else {
        strcpy((char *) &(wifi_config.sta.ssid[0]), APP_WIFI_SSID);
        ESP_LOGD(WC_TAG, "SSID setted from flash config");
    }
    value = get_cfg_value(CFG_SSID_PASSWORD);
    if (value != NULL) {
        strcpy((char *) &(wifi_config.sta.password[0]), value);
        ESP_LOGD(WC_TAG, "Password setted from json config");
    } else {
        strcpy((char *) &(wifi_config.sta.password[0]), APP_WIFI_PASS);
        ESP_LOGD(WC_TAG, "Password setted from flash config");
    }

    ESP_LOGI(WC_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void msgs_get_cb(void* arg)
{
    ESP_LOGD(WC_TAG, "Recieve msgs fired");
    bool isempty = h2pc_im_locked_waiting();
    if (isempty) {
        if (locked_CHK_STATE(HOST_CONNECTED_BIT))
            locked_SET_STATE(MODE_RECIEVE_MSG);
    }
}

void msgs_send_cb(void* arg)
{
    ESP_LOGD(WC_TAG, "Send msgs fired");
    bool isnempty = h2pc_om_locked_waiting();
    if (isnempty) {
        if (locked_CHK_STATE(HOST_CONNECTED_BIT))
            locked_SET_STATE(MODE_SEND_MSG);
    }
}

static void refresh_ext_pow() {
    int level = gpio_get_level(IN_EXT_PWR);

    locked_set_ext_power(level > 0);
}

void broadcast_cb(void* arg)
{
    ESP_LOGI(WC_TAG, "Broadcast fired");

    refresh_ext_pow();

    if (locked_CHK_STATE(AUTHORIZED_BIT)) {

        bool need_to_send_broadcast = false;

        if (LOCK_MUX) {
            if (ENABLE_BROADCAST) {
                struct timeval tv;
                gettimeofday(&tv, NULL);

                if (tv.tv_sec < BROADCAST_TIME) {
                    need_to_send_broadcast = true;
                } else
                if ((tv.tv_sec - BROADCAST_TIME) >= 3000) { // 5 min is over
                    need_to_send_broadcast = true;
                } else
                if ((tv.tv_sec - BROADCAST_TIME) >= 30) {   // 0.5 minute is over
                    if (fabs(LST_BRD_STATUS_T - TEMPERATURE) >= 0.25)
                        need_to_send_broadcast = true;
                    else
                    if (LST_BRD_STATUS_TARGET_T != T_MODES[T_MODE_CUR])
                        need_to_send_broadcast = true;
                    else
                    if (LST_BRD_STATUS_EXT_POW != EXT_POWER)
                        need_to_send_broadcast = true;
                    else
                    if (LST_BRD_STATUS_HEATING != HEATING)
                        need_to_send_broadcast = true;
                }
            }
            UNLOCK_MUX;
        }
        if (need_to_send_broadcast)
            locked_SET_STATE(MODE_SEND_BROAD);
    }
}

void meas_temp_cb(void* arg)
{
    ESP_LOGD(WC_TAG, "Measure temperature fired");
    locked_SET_STATE(MODE_MEASURE_T);
}

void blink_led_cb(void * arg) {
    CUR_BLINK_PATTERN_LOC <<= 1;
    if (CUR_BLINK_PATTERN_LOC == 0)
        CUR_BLINK_PATTERN_LOC = 1;
    if (CUR_BLINK_PATTERN_LOC & locked_get_blink_pattern())
        gpio_set_level(OUT_LED, OUT_ON);
    else
        gpio_set_level(OUT_LED, OUT_OFF);
}

#define MODE_OP_GET 0
#define MODE_OP_SET 1
#define MODE_OP_RUN 2

static void msg_send_modeid_value(cJSON * omsg, uint8_t modeid, float value) {
    cJSON_AddNumberToObject(omsg, JSON_RPC_MODE_ID, modeid);
    cJSON_AddNumberToObject(omsg, JSON_RPC_MODE_VALUE, value);
}

bool mode_op(int _modeop, cJSON * omsg, const cJSON * params) {
    if (params) {
        cJSON * mname = cJSON_GetObjectItem(params,   JSON_RPC_MODE_ID);   //selected mode
        if (mname) {
            uint8_t modeid = (uint8_t) mname->valueint;

            if (modeid < MAX_T_MODES) {
                switch (_modeop) {
                    case MODE_OP_GET: {
                        float v = locked_get_t_mode(modeid);
                        msg_send_modeid_value(omsg, modeid, v);
                        return true;
                    }
                    case MODE_OP_SET: {
                        cJSON * mvalue = cJSON_GetObjectItem(params,   JSON_RPC_MODE_VALUE);   //new value
                        if (mvalue) {
                            locked_set_t_mode(modeid, mvalue->valuedouble);
                            float v = locked_get_t_mode(modeid);
                            msg_send_modeid_value(omsg, modeid, v);
                            return true;
                        }
                        break;
                    }
                    case MODE_OP_RUN: {
                        locked_set_cur_t_mode_id(modeid);
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

static void send_is_broad(const char * src_s, cJSON * omsg) {
    cJSON_AddBoolToObject(omsg, JSON_RPC_IS_BROAD,    locked_get_is_broadcast());
    h2pc_om_add_msg(JSON_RPC_IS_BROAD, src_s, omsg);
}

#define STATUS_T_SEND     0x01
#define STATUS_TT_SEND    0x02
#define STATUS_MID_SEND   0x04
#define STATUS_HYST_SEND  0x08
#define STATUS_EP_SEND    0x10
#define STATUS_HEAT_SEND  0x20
#define MODES_SEND        0x40
#define ISBROAD_SEND      0x80

#define STATUS_FULL_SEND  (STATUS_T_SEND|STATUS_TT_SEND|STATUS_MID_SEND|STATUS_HYST_SEND|STATUS_EP_SEND|STATUS_HEAT_SEND)

static void send_status(const char * src_s, uint8_t mask, cJSON * omsg) {
    if (LOCK_MUX) {

        if (((mask&STATUS_T_SEND) != 0) && locked_CHK_STATE(MODE_HAS_T))
            cJSON_AddNumberToObject(omsg,  JSON_RPC_T,        TEMPERATURE);
        if ((mask&(STATUS_TT_SEND|STATUS_MID_SEND)) != 0) {
            cJSON_AddNumberToObject(omsg,  JSON_RPC_TARGET_T, T_MODES[T_MODE_CUR]);
            cJSON_AddNumberToObject(omsg,  JSON_RPC_MODE_ID,  T_MODE_CUR);
        }
        if ((mask&STATUS_HYST_SEND) != 0)
            cJSON_AddNumberToObject(omsg,  JSON_RPC_HYST,     T_HYST);
        if ((mask&STATUS_EP_SEND) != 0)
            cJSON_AddBoolToObject(omsg, JSON_RPC_EXT_POW,     EXT_POWER);
        if ((mask&STATUS_HEAT_SEND) != 0)
            cJSON_AddBoolToObject(omsg, JSON_RPC_HEAT,        HEATING);
        UNLOCK_MUX;
    }
    h2pc_om_add_msg(JSON_RPC_STATUS, src_s, omsg);
}

#define MAX_STATUS_TARGET_NAME_LEN 32

typedef struct status_target_t {
    char name[MAX_STATUS_TARGET_NAME_LEN];
    uint8_t bitmask;
} status_target;

#define MAX_STATUS_TARGETS 10

static status_target status_targets[MAX_STATUS_TARGETS] = { 0 };
static int status_target_offet = -1;

static void resetStatusTargetLocs() {
    for (int i = 0; i < MAX_STATUS_TARGETS; i++) {
        status_targets[i].bitmask = 0;
    }
}

static int getStatusTargetLoc(const char * trg) {
    for (int i = 0; i < MAX_STATUS_TARGETS; i++) {
        if (strcmp(&(status_targets[i].name[0]), trg) == 0) {
            return i;
        }
    }
    status_target_offet++;
    if (status_target_offet >= MAX_STATUS_TARGETS) status_target_offet = 0;
    memset(&(status_targets[status_target_offet].name[0]), 0, MAX_STATUS_TARGET_NAME_LEN);
    strcpy(&(status_targets[status_target_offet].name[0]), trg);
    status_targets[status_target_offet].bitmask = 0;
    return status_target_offet;
}

static void setStatusTargetBit(int trg, uint8_t bitmask) {
    status_targets[trg].bitmask |= bitmask;
}

bool on_incoming_msg(const cJSON * src, const cJSON * kind, const cJSON * iparams, const cJSON * msg_id) {
    char * src_s = src->valuestring;
    if (strcmp(src_s, device_name) != 0) {
        int trgloc = getStatusTargetLoc(src_s);

        char * kind_s = kind->valuestring;
        /* broadcasting */
        if (strcmp(JSON_RPC_START_BROAD, kind_s) == 0) {
            locked_set_is_broadcast(true);

            setStatusTargetBit(trgloc, ISBROAD_SEND);
        } else
        if (strcmp(JSON_RPC_STOP_BROAD, kind_s) == 0) {
            locked_set_is_broadcast(false);

            setStatusTargetBit(trgloc, ISBROAD_SEND);
        } else
        if (strcmp(JSON_RPC_IS_BROAD, kind_s) == 0) {
            setStatusTargetBit(trgloc, ISBROAD_SEND);
        } else
        /* getters for status */
        if ((strcmp(JSON_RPC_STATUS, kind_s) == 0) &&
            (cJSON_GetArraySize(iparams) == 0)) {
            setStatusTargetBit(trgloc, STATUS_FULL_SEND);
        } else
        if (strcmp(JSON_RPC_GET_T, kind_s) == 0) {
            setStatusTargetBit(trgloc, STATUS_T_SEND);
        } else
        if (strcmp(JSON_RPC_CUR_MODE, kind_s) == 0) {
            setStatusTargetBit(trgloc, STATUS_MID_SEND);
        } else
        if (strcmp(JSON_RPC_GET_HYST, kind_s) == 0) {
            setStatusTargetBit(trgloc, STATUS_HYST_SEND);
        } else
        if (strcmp(JSON_RPC_GET_EXT_POW, kind_s) == 0) {
            setStatusTargetBit(trgloc, STATUS_EP_SEND);
        } else
        /* setters for status */
        if (strcmp(JSON_RPC_RUN_MODE, kind_s) == 0) {
            mode_op(MODE_OP_RUN, NULL, iparams);
            setStatusTargetBit(trgloc, STATUS_MID_SEND);
        } else
        if (strcmp(JSON_RPC_SET_HYST, kind_s) == 0) {
            cJSON * hystp = cJSON_GetObjectItem(iparams,   JSON_RPC_HYST);
            if (hystp) {
                float v = hystp->valuedouble;
                locked_set_hyst(v);
            }
            setStatusTargetBit(trgloc, STATUS_HYST_SEND);
        } else
        /* getters for modes */
        if ((strcmp(JSON_RPC_MODE_LIST, kind_s) == 0) &&
            (cJSON_GetArraySize(iparams) == 0)) {

            setStatusTargetBit(trgloc, MODES_SEND);
        } else
        if (strcmp(JSON_RPC_GET_MODE, kind_s) == 0) {
            cJSON * oparams = cJSON_CreateObject();
            bool res = mode_op(MODE_OP_GET, oparams, iparams);
            h2pc_om_add_msg_res(JSON_RPC_MODE, src_s, oparams, res);
        } else
        /* setter for mode */
        if (strcmp(JSON_RPC_SET_MODE, kind_s) == 0) {
            cJSON * oparams = cJSON_CreateObject();
            bool res = mode_op(MODE_OP_SET, oparams, iparams);
            h2pc_om_add_msg_res(JSON_RPC_MODE, src_s, oparams, res);
        }

    }
    return true;
}

static void consumeStatusTargetLocs() {
    for (int i = 0; i < MAX_STATUS_TARGETS; ++i) {
        const char * src_s = status_targets[i].name;
        if (strlen(src_s) > 0) {
            uint8_t status = status_targets[i].bitmask;

            if ((status & STATUS_FULL_SEND) != 0) {
                cJSON * oparams = cJSON_CreateObject();
                send_status(src_s, status, oparams);
            }

            if (status & ISBROAD_SEND) {
                cJSON * oparams = cJSON_CreateObject();
                send_is_broad(src_s, oparams);
            }

            if (status & MODES_SEND) {
                cJSON * oparams = cJSON_CreateObject();
                cJSON * modes = cJSON_CreateArray();

                if (LOCK_MUX) {
                    for (int i = 0; i < MAX_T_MODES; i++) {
                        cJSON * cfg_item = cJSON_CreateNumber(T_MODES[i]);
                        cJSON_AddItemToArray(modes, cfg_item);
                    }
                    UNLOCK_MUX;
                }

                cJSON_AddItemToObject(oparams, JSON_RPC_MODE_VALUES, modes);

                h2pc_om_add_msg(JSON_RPC_MODE_LIST, src_s, oparams);
            }
        }
    }
}

static void send_broadcast() {
    locked_CLR_STATE(MODE_SEND_BROAD);

    if (LOCK_MUX) {

        if (ENABLE_BROADCAST) {
            struct timeval tv;
            gettimeofday(&tv, NULL);

            BROADCAST_TIME = tv.tv_sec;
            LST_BRD_STATUS_EXT_POW  = EXT_POWER;
            LST_BRD_STATUS_HEATING = HEATING;
            LST_BRD_STATUS_T = TEMPERATURE;
            LST_BRD_STATUS_TARGET_T = T_MODES[T_MODE_CUR];

            cJSON * oparams = cJSON_CreateObject();
            send_status(JSON_RPC_TARGET_BROADCAST, STATUS_FULL_SEND, oparams);
        }

        UNLOCK_MUX;
    }
}

static void measure_ext_power() {

    if (locked_syn_ext_power()) {
        if (locked_CHK_STATE(AUTHORIZED_BIT)) {
            cJSON * oparams = cJSON_CreateObject();
            cJSON_AddBoolToObject(oparams, JSON_RPC_EXT_POW, locked_get_ext_power());
            h2pc_om_add_msg(JSON_RPC_STATUS, JSON_RPC_TARGET_BROADCAST, oparams);
        }
    }

}

static void measure_heating() {

    if (locked_syn_heating()) {
        if (locked_CHK_STATE(AUTHORIZED_BIT)) {
            cJSON * oparams = cJSON_CreateObject();
            cJSON_AddBoolToObject(oparams, JSON_RPC_HEAT, locked_get_heating());
            h2pc_om_add_msg(JSON_RPC_STATUS, JSON_RPC_TARGET_BROADCAST, oparams);
        }
    }

}

static void measure_temperature() {
    static uint8_t sensor_data[8] = {0};

    ESP_LOGI(JSON_CFG, "OWI temperature measuring");

    if (OWI_DetectPresence(OW_PNUM)) {

        OWI_SendByte(0xCC, OW_PNUM);
        OWI_SendByte(0x44, OW_PNUM);

        vTaskDelay(configTICK_RATE_HZ * 3 / 4);

        uint8_t c = 0;
        bool ready = true;
        while ((OWI_DetectFinished(OW_PNUM) == 0)) {
            ESP_LOGI(JSON_CFG, "OWI measuring timeout");
            vTaskDelay(configTICK_RATE_HZ / 4);
            c++;
            if (c == 4) {
                ready = false;
                break;
            }
        }

        if (!ready)
            ESP_LOGI(JSON_CFG, "OWI is not ready");

        // Read Scratch memory area
        if (ready && (OWI_DetectPresence(OW_PNUM) != 0)) {

            OWI_SendByte(0xCC, OW_PNUM);
            OWI_SendByte(0xBE, OW_PNUM);

            uint8_t crc8 = 0;
            for (uint8_t ee = 0; ee < 8; ++ee) {
                sensor_data[ee] = OWI_ReceiveByte(OW_PNUM);
                crc8 = OWI_ComputeCRC8(sensor_data[ee], crc8);
            }
            sensor_data[8] = OWI_ReceiveByte( OW_PNUM);
            if (sensor_data[8] == crc8) {
                //1111 1110 0110 1111

                uint16_t tt = (sensor_data[1] << 8) | sensor_data[0];
                int8_t sign;
                float value;
		        if (sensor_data[1] & 0x08) {
			        sign = 1;
			        tt -= 1;
			        tt = ~tt;
			    } else {
			        sign = 0;
		        }
		        value = (float)((tt >> 4) & 0xff);
		        if (tt & 0x0001) value += 0.0625;
		        if (tt & 0x0002) value += 0.125;
		        if (tt & 0x0004) value += 0.25;
		        if (tt & 0x0008) value += 0.5;
                if (sign) value = -value;
                locked_set_temperature(value);

                locked_SET_STATE(MODE_HAS_T);
                locked_CLR_STATE(MODE_MEASURE_T);
            }

        }
    }
}

static void update_relay() {
    if (locked_get_ext_power()) {
        if (locked_CHK_STATE(MODE_HAS_T)) {
            float temp  = locked_get_temperature();
            float mtemp = locked_get_cur_t_mode_value();
            float htemp = locked_get_hyst();
            float dt = temp - mtemp;

            if (dt > htemp) {
                gpio_set_level(OUT_SWITCH, SWITCH_OUT_OFF);
                locked_set_heating(false);
            }
            else
            if (dt < -htemp) {
                gpio_set_level(OUT_SWITCH, SWITCH_OUT_ON);
                locked_set_heating(true);
            }
        }
    } else {
        // for battery saving
        gpio_set_level(OUT_SWITCH, SWITCH_OUT_OFF);
        locked_set_heating(false);
    }
}

static void ext_gpio_isr_handler(void* arg)
{
    refresh_ext_pow();
}

static void init_board_gpio() {
    gpio_install_isr_service(0);
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL << IN_EXT_PWR);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&gpio_conf);
    gpio_isr_handler_add(IN_EXT_PWR, ext_gpio_isr_handler, NULL);

    gpio_pad_select_gpio(OUT_SWITCH);
    gpio_set_direction(OUT_SWITCH, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_SWITCH, SWITCH_OUT_OFF);

    gpio_pad_select_gpio(OUT_LED);
    gpio_set_direction(OUT_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_LED, OUT_OFF);

    gpio_pad_select_gpio(OW_PNUM);
    gpio_set_direction(OW_PNUM, GPIO_MODE_OUTPUT);
    gpio_set_level(OW_PNUM, OUT_OFF);
}

static bool nvs_write_tvars(nvs_handle h) {
    size_t sz = sizeof(float);

    if (nvs_set_blob(h, JSON_RPC_HYST, &T_HYST, sz) != ESP_OK) return false;
    sz = sizeof(float) * (MAX_T_MODES);
    if (nvs_set_blob(h, JSON_RPC_MODE, &(T_MODES[0]), sz) != ESP_OK) return false;
    if (nvs_set_u8(h, JSON_RPC_CUR_MODE, T_MODE_CUR) != ESP_OK) return false;
    if (nvs_set_u8(h, JSON_RPC_IS_BROAD, ENABLE_BROADCAST) != ESP_OK) return false;
    nvs_commit(h);
    return true;
}

static void nvs_read_tvars(nvs_handle h) {
    size_t sz = sizeof(float);

    if (nvs_get_blob(h, JSON_RPC_HYST, &T_HYST, &sz) != ESP_OK) return;
    sz = sizeof(float) * (MAX_T_MODES);
    if (nvs_get_blob(h, JSON_RPC_MODE, &(T_MODES[0]), &sz) != ESP_OK) return;
    if (nvs_get_u8(h, JSON_RPC_CUR_MODE, &T_MODE_CUR) != ESP_OK) return;
    uint8_t v = 0;
    if (nvs_get_u8(h, JSON_RPC_IS_BROAD, &v) != ESP_OK) return;
    ENABLE_BROADCAST = v;
}

static void check_h2pc_errors() {
    if (locked_CHK_STATE(WIFI_CONNECTED_BIT|HOST_CONNECTED_BIT)) {
        if (h2pc_get_connected()) {
            if (h2pc_get_protocol_errors_cnt() > 0) {
                if (h2pc_get_last_error() == REST_ERR_NO_SUCH_SESSION) {
                    locked_CLR_STATE(AUTHORIZED_BIT);
                    locked_SET_STATE(MODE_AUTH);
                } else
                    disconnect_host();
            }
        } else {
            disconnect_host();
        }
    }
}

#define MAIN_TASK_LOOP_DELAY 200

static void main_task(void *args)
{
    nvs_handle my_handle;
    esp_err_t err;
    cJSON * loc_cfg = NULL;

    err = nvs_open(DEVICE_CONFIG, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        size_t required_size;
        err = nvs_get_str(my_handle, JSON_CFG, NULL, &required_size);
        if (err == ESP_OK) {
            char * cfg_str = malloc(required_size);
            nvs_get_str(my_handle, JSON_CFG, cfg_str, &required_size);
            loc_cfg = cJSON_Parse(cfg_str);
            free(cfg_str);
            ESP_LOGD(JSON_CFG, "JSON cfg founded");
            #ifdef LOG_DEBUG
            esp_log_buffer_char(JSON_CFG, cfg_str, strlen(cfg_str));
            #endif
        }
        nvs_read_tvars(my_handle);
        T_VARS_CHANGED = false;
    }

    if (loc_cfg == NULL) {
        loc_cfg = cJSON_CreateArray();
        cJSON * cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_DEVICE_NAME), mac_str);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_USER_NAME), HTTP2_SERVER_NAME);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_USER_PASSWORD), HTTP2_SERVER_PASS);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_HOST_NAME), HTTP2_SERVER_URI);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_SSID_NAME), APP_WIFI_SSID);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_SSID_PASSWORD), APP_WIFI_PASS);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
    }

    locked_set_blink_pattern(BLINK_LED_BLE);

    error_t ret = initialize_ble(loc_cfg);
    cJSON_Delete(loc_cfg);
    if (ret == OK) {
        start_ble_config_round();
        while ( ble_config_proceed() ) {
            vTaskDelay(1000);
        }
        stop_ble_config_round();

        if (WC_CFG_VALUES != NULL) {
            char * cfg_str = cJSON_PrintUnformatted(WC_CFG_VALUES);
            nvs_set_str(my_handle, JSON_CFG, cfg_str);
            nvs_commit(my_handle);

            #ifdef LOG_DEBUG
            esp_log_buffer_char(JSON_CFG, cfg_str, strlen(cfg_str));
            #endif

            cJSON_free(cfg_str);
        }
    }
    nvs_close(my_handle);


    ESP_ERROR_CHECK(h2pc_initialize(H2PC_MODE_MESSAGING));
    initialise_wifi();

    /* init timers */
    esp_timer_create_args_t timer_args;

    timer_args.callback = &msgs_get_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &msgs_recieve));

    timer_args.callback = &msgs_send_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &msgs_send));

    timer_args.callback = &meas_temp_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &meas_temp));

    timer_args.callback = &broadcast_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &broadcast));

    /* start timers */
    esp_timer_start_periodic(msgs_recieve, GET_MSG_TIMER_DELTA);
    esp_timer_start_periodic(msgs_send, SEND_MSG_TIMER_DELTA);
    esp_timer_start_periodic(meas_temp, MEASURE_TEMPERATURE_TIMER_DELTA);
    esp_timer_start_periodic(broadcast, BROADCAST_TIMER_DELTA);

    int connectDelay = 0;
    int wifiDisconnectedTime = 0;

    while (1)
    {
        // ESP_LOGI(WC_TAG, "New step. states: %d", locked_GET_STATES());

        switch (locked_GET_STATES() & 0x000007)
        {
        case 0:
            locked_set_blink_pattern(BLINK_LED_WIFI_START);
            break;
        case (uint32_t)(WIFI_CONNECTED_BIT):
            locked_set_blink_pattern(BLINK_LED_WIFI_CONNECTED);
            break;
        case (uint32_t)(WIFI_CONNECTED_BIT|HOST_CONNECTED_BIT):
            locked_set_blink_pattern(BLINK_LED_HOST_CONNECTED);
            break;
        case (uint32_t)(WIFI_CONNECTED_BIT|HOST_CONNECTED_BIT|AUTHORIZED_BIT):
            locked_set_blink_pattern(BLINK_LED_AUTHORIZED);
            break;
        default:
            locked_set_blink_pattern(BLINK_LED_ERROR);
            break;
        }

        if (locked_CHK_STATE(WIFI_CONNECTED_BIT)) {

            wifiDisconnectedTime = 0;

            if (locked_CHK_STATE(MODE_SETIME)) {
                /* Set current time: proper system time is required for TLS based
                 * certificate verification.
                 */
                set_time();
                locked_CLR_STATE(MODE_SETIME);
            }


            if (locked_CHK_STATE(HOST_CONNECTED_BIT)) {

                /* authorize the device on server */
                if (locked_CHK_STATE(MODE_AUTH)) {
                    send_authorize();
                    check_h2pc_errors();
                }
                /* gathering incoming msgs from server */
                if (locked_CHK_STATE(MODE_RECIEVE_MSG)) {
                    esp_timer_stop(msgs_recieve);
                    recieve_msgs();
                    check_h2pc_errors();
                    if (locked_get_ext_power())
                        esp_timer_start_periodic(msgs_recieve, GET_MSG_TIMER_DELTA);
                    else
                        esp_timer_start_periodic(msgs_recieve, GET_MSG_NO_EP_TIMER_DELTA);
                }
                /* proceed incoming messages */
                resetStatusTargetLocs();
                h2pc_im_proceed(&on_incoming_msg, 16);
                consumeStatusTargetLocs();

                if (locked_CHK_STATE(MODE_SEND_BROAD)) {
                    send_broadcast();
                }

                /* send outgoing messages */
                if (locked_CHK_STATE(MODE_SEND_MSG)) {
                    esp_timer_stop(msgs_send);
                    send_msgs();
                    check_h2pc_errors();
                    if (locked_get_ext_power())
                        esp_timer_start_periodic(msgs_send, SEND_MSG_TIMER_DELTA);
                    else
                        esp_timer_start_periodic(msgs_send, SEND_MSG_NO_EP_TIMER_DELTA);
                }

            } else {

                connectDelay -= MAIN_TASK_LOOP_DELAY;

                if (connectDelay <= 0) {

                    connect_to_http2();

                    if (connect_errors) {
                        switch (connect_errors)
                        {
                        case 11:
                            connectDelay = 300 * configTICK_RATE_HZ; // 5 minutes
                            break;
                        case 12:
                            assert(ESP_ERR_INVALID_STATE); // drop to deep reload if no connection to host over 15 minutes
                            break;
                        default:
                            connectDelay = connect_errors * 10 * configTICK_RATE_HZ;
                            break;
                        }
                    } else
                        connectDelay = 0;

                }

            }

        } else {
            wifiDisconnectedTime += MAIN_TASK_LOOP_DELAY;

            if (wifiDisconnectedTime > 900000)
                assert(ESP_ERR_INVALID_STATE); // drop to deep reload if no connection to AP over 15 minutes

            connectDelay -= MAIN_TASK_LOOP_DELAY;

            if ((connectDelay <= 0) && (wifi_connect_errors)) {

                wifi_connect_errors = 0;
                ESP_ERROR_CHECK(esp_wifi_connect());

                connectDelay = 30 * configTICK_RATE_HZ; // 30 sec timeout between two wifi connection attempts
            }
        }
        /* measure temperature open/close relay */
        if (locked_CHK_STATE(MODE_MEASURE_T)) {
            esp_timer_stop(meas_temp);
            measure_temperature();
            if (locked_get_ext_power())
                esp_timer_start_periodic(meas_temp, MEASURE_TEMPERATURE_TIMER_DELTA);
            else
                esp_timer_start_periodic(meas_temp, MEASURE_TEMPERATURE_NO_EP_TIMER_DELTA);
        }
        if (LOCK_MUX) {
            if (T_VARS_CHANGED) {
                err = nvs_open(DEVICE_CONFIG, NVS_READWRITE, &my_handle);
                if (err == ESP_OK) {
                    if (nvs_write_tvars(my_handle))
                        T_VARS_CHANGED = false;
                    nvs_close(my_handle);
                }
            }
            UNLOCK_MUX;
        }
        /* measure ext power */
        measure_ext_power();
        update_relay();
        measure_heating();

        vTaskDelay(MAIN_TASK_LOOP_DELAY);
    }

    finalize_app();

    vTaskDelete(NULL);
}


void finalize_app()
{
    disconnect_host();

    esp_timer_stop(msgs_send);
    esp_timer_stop(msgs_recieve);
    esp_timer_stop(meas_temp);
    esp_timer_stop(broadcast);


    h2pc_finalize();

    if (device_meta_data) free(device_meta_data);
}

static char DEVICE_CHAR [] = "00000000";

void app_main()
{
    esp_err_t err;

    client_state = xEventGroupCreate();
    vars_mux = xSemaphoreCreateRecursiveMutex();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* generate mac address and device metadata */
    uint8_t sta_mac[6];
    ESP_ERROR_CHECK( esp_efuse_mac_get_default(sta_mac) );
    for (int i = 0; i < 6; i++) {
        mac_str[i<<1] = UPPER_XDIGITS[(sta_mac[i] >> 4) & 0x0f];
        mac_str[(i<<1) + 1] = UPPER_XDIGITS[(sta_mac[i] & 0x0f)];
    }

    mac_str[12] = 0;

    DEVICE_CHAR[4] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 12) & 0x000f)];
    DEVICE_CHAR[5] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 8) & 0x000f)];
    DEVICE_CHAR[6] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 4) & 0x000f)];
    DEVICE_CHAR[7] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID) & 0x000f)];
    device_meta_data = cJSON_CreateObject();
    cJSON_AddItemToObject(device_meta_data, JSON_BLE_CHAR, cJSON_CreateStringReference(DEVICE_CHAR));

    init_board_gpio();

    /* init blink timer */
    locked_set_blink_pattern(BLINK_LED_OFF);
    esp_timer_create_args_t timer_args;
    timer_args.callback = &blink_led_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_led));
    esp_timer_start_periodic(blink_led, LED_BLINK_TIMER_DELTA);

    /* start main task */
    xTaskCreate(&main_task, "main_task", (1024 * 48), NULL, 5, NULL);
}
