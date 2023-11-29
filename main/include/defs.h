// Copyright 2023 Medvedkov Ilya
//
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFS_H_
#define DEFS_H_

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <cJSON.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#if CONFIG_LOG_DEFAULT_LEVEL > 3

#define LOG_DEBUG

#endif

#define DEVICE_CONFIG   "device_config"

#define OUT_LED         GPIO_NUM_5
#define OUT_SWITCH      GPIO_NUM_25
#define OW_PNUM         GPIO_NUM_19

#define IN_EXT_PWR      GPIO_NUM_33

extern const char * WC_TAG;

extern const char const DEVICE_NAME [];

#endif
