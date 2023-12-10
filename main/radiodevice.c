/* HTTP2 Web Radio Listener Device

   Part of WCWebCamServer, LiteSound projects

   Copyright (c) 2023 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "defs.h"

#include "wcprotocol.h"
#include "http2_protoclient.h"
#include "wch2pcapp.h"

#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
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

/* h2pca */
static h2pca_config app_cfg;
static h2pca_status * h2pca_app;

// additional modes for h2pcapp
const int  MODE_MEASURE_T    = BIT10;
const int  MODE_SEND_T       = BIT11;
const int  MODE_HAS_T        = BIT12;
// recieve/send msg. is need to send msg from pool to server
const int  MODE_T_VARS_RW    = BIT13;
const int  MODE_SEND_BROAD   = BIT14;

/* timers */
static esp_timer_handle_t blink_led;
#define LOC_SEND_MSG_TIMER_DELTA               1000000
#define LOC_SEND_MSG_NO_EP_TIMER_DELTA         2000000
#define LOC_GET_MSG_TIMER_DELTA                4000000
#define LOC_GET_MSG_NO_EP_TIMER_DELTA          8000000
#define BROADCAST_TIMER_DELTA                  15000000
#define MEASURE_TEMPERATURE_TIMER_DELTA        10000000
#define MEASURE_TEMPERATURE_NO_EP_TIMER_DELTA  30000000
#define LED_BLINK_TIMER_DELTA                  150000

/* Blink LED modes */
#define BLINK_LED_OFF             0x0000
#define BLINK_LED_ON              0xFFFF
#define BLINK_LED_BLE             0x0033
#define BLINK_LED_WIFI_START      0xFFFF
#define BLINK_LED_WIFI_CONNECTED  0xFFF8
#define BLINK_LED_HOST_CONNECTED  0xF0F0
#define BLINK_LED_AUTHORIZED      0x0001
#define BLINK_LED_ERROR           0xAAAA

/* JSON-RPC device metadata */

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

/* global vars */
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

/* internal relay logic */

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

        if (((mask&STATUS_T_SEND) != 0) && h2pca_locked_CHK_STATE(MODE_HAS_T))
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
    if (strcmp(src_s, h2pca_get_status()->device_name) != 0) {
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

static void refresh_ext_pow() {
    int level = gpio_get_level(IN_EXT_PWR);

    locked_set_ext_power(level > 0);
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

static void send_broadcast() {
    h2pca_locked_CLR_STATE(MODE_SEND_BROAD);

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
        if (h2pca_locked_CHK_STATE(AUTHORIZED_BIT)) {
            cJSON * oparams = cJSON_CreateObject();
            cJSON_AddBoolToObject(oparams, JSON_RPC_EXT_POW, locked_get_ext_power());
            h2pc_om_add_msg(JSON_RPC_STATUS, JSON_RPC_TARGET_BROADCAST, oparams);
        }
    }

}

static void measure_heating() {

    if (locked_syn_heating()) {
        if (h2pca_locked_CHK_STATE(AUTHORIZED_BIT)) {
            cJSON * oparams = cJSON_CreateObject();
            cJSON_AddBoolToObject(oparams, JSON_RPC_HEAT, locked_get_heating());
            h2pc_om_add_msg(JSON_RPC_STATUS, JSON_RPC_TARGET_BROADCAST, oparams);
        }
    }

}

static void measure_temperature() {
    static uint8_t sensor_data[8] = {0};

    ESP_LOGI(WC_TAG, "OWI temperature measuring");

    if (OWI_DetectPresence(OW_PNUM)) {

        OWI_SendByte(0xCC, OW_PNUM);
        OWI_SendByte(0x44, OW_PNUM);

        vTaskDelay(configTICK_RATE_HZ * 3 / 4);

        uint8_t c = 0;
        bool ready = true;
        while ((OWI_DetectFinished(OW_PNUM) == 0)) {
            ESP_LOGI(WC_TAG, "OWI measuring timeout");
            vTaskDelay(configTICK_RATE_HZ / 4);
            c++;
            if (c == 4) {
                ready = false;
                break;
            }
        }

        if (!ready)
            ESP_LOGI(WC_TAG, "OWI is not ready");

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

                h2pca_locked_SET_STATE(MODE_HAS_T);
                h2pca_locked_CLR_STATE(MODE_MEASURE_T);
            }

        }
    }
}

static void update_relay() {
    if (locked_get_ext_power()) {
        if (h2pca_locked_CHK_STATE(MODE_HAS_T)) {
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
    T_VARS_CHANGED = false;
}

/* h2pca callbacks */

static void on_disconnect_host() {
    locked_set_blink_pattern(BLINK_LED_ERROR);
}

static void on_wifi_init() {
    locked_set_blink_pattern(BLINK_LED_WIFI_START);
}

static void on_ble_cfg_start() {
    locked_set_blink_pattern(BLINK_LED_BLE);
}

static void on_begin_step() {
    switch (h2pca_locked_GET_STATES() & 0x000007)
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

    if (locked_get_ext_power()) {
        app_cfg.recv_msgs_period = LOC_GET_MSG_TIMER_DELTA;
        app_cfg.send_msgs_period = LOC_SEND_MSG_TIMER_DELTA;
    } else {
        app_cfg.recv_msgs_period = LOC_GET_MSG_NO_EP_TIMER_DELTA;
        app_cfg.send_msgs_period = LOC_SEND_MSG_NO_EP_TIMER_DELTA;
    }
}

static void on_finish_step() {
    if (LOCK_MUX) {
        if (T_VARS_CHANGED) {
            nvs_handle my_handle;
            esp_err_t err;

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
}

static void sync_measure_t_task_cb(h2pca_task_id id,
                                     h2pca_state cur_state,
                                     void * user_data,
                                     uint32_t * restart_period) {
    measure_temperature();
    if (locked_get_ext_power())
        *restart_period = MEASURE_TEMPERATURE_TIMER_DELTA;
    else
        *restart_period = MEASURE_TEMPERATURE_NO_EP_TIMER_DELTA;
}

static void broadcast_cb(h2pca_task_id id, void * user_data)
{
    refresh_ext_pow();

    if (h2pca_locked_CHK_STATE(AUTHORIZED_BIT)) {

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
            h2pca_locked_SET_STATE(MODE_SEND_BROAD);
    }
}

static void sync_broadcast_task_cb(h2pca_task_id id,
                                     h2pca_state cur_state,
                                     void * user_data,
                                     uint32_t * restart_period) {
    send_broadcast();
}

void app_main()
{
    esp_err_t err = h2pca_init_cfg(&app_cfg);
    ESP_ERROR_CHECK(err);

    app_cfg.LOG_TAG = WC_TAG;

    app_cfg.h2pcmode = H2PC_MODE_MESSAGING;
    app_cfg.recv_msgs_period = LOC_GET_MSG_TIMER_DELTA;
    app_cfg.send_msgs_period = LOC_SEND_MSG_TIMER_DELTA;
    app_cfg.inmsgs_proceed_chunk = 16;

    app_cfg.on_wifi_init = &on_wifi_init;
    app_cfg.on_disconnect = &on_disconnect_host;
    app_cfg.on_read_nvs = &nvs_read_tvars;
    app_cfg.on_ble_cfg_start = &on_ble_cfg_start;
    app_cfg.on_begin_step = &on_begin_step;
    app_cfg.on_before_inmsgs = &resetStatusTargetLocs;
    app_cfg.on_after_inmsgs = &consumeStatusTargetLocs;
    app_cfg.on_next_inmsg = &on_incoming_msg;
    app_cfg.on_finish_step = &on_finish_step;

    h2pca_task * tsk;

    tsk = h2pca_new_task("Measure", 1, NULL, &err);
    ESP_ERROR_CHECK(err);
    tsk->on_sync = &sync_measure_t_task_cb;
    tsk->apply_bitmask = MODE_MEASURE_T;
    tsk->req_bitmask = 0;
    tsk->period = MEASURE_TEMPERATURE_TIMER_DELTA;
    ESP_ERROR_CHECK(h2pca_task_pool_add_task(&(app_cfg.tasks), tsk));

    tsk = h2pca_new_task("Broadcast", 2, NULL, &err);
    ESP_ERROR_CHECK(err);
    tsk->on_time = &broadcast_cb;
    tsk->on_sync = &sync_broadcast_task_cb;
    tsk->apply_bitmask = MODE_SEND_BROAD;
    tsk->req_bitmask = HOST_CONNECTED_BIT|WIFI_CONNECTED_BIT;
    tsk->period = BROADCAST_TIMER_DELTA;
    ESP_ERROR_CHECK(h2pca_task_pool_add_task(&(app_cfg.tasks), tsk));

    h2pca_app = h2pca_init(&app_cfg, &err);

    if (h2pca_app == NULL) {
        ESP_LOGE(WC_TAG, "Application not initialized, error code %d", err);
        ESP_ERROR_CHECK(err);
    }

    vars_mux = xSemaphoreCreateRecursiveMutex();

    init_board_gpio();

    /* init blink timer */
    locked_set_blink_pattern(BLINK_LED_OFF);
    esp_timer_create_args_t timer_args;
    timer_args.callback = &blink_led_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_led));
    esp_timer_start_periodic(blink_led, LED_BLINK_TIMER_DELTA);

    /* start main task */
    h2pca_start(0);
}
