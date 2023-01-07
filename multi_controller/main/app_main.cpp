/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app_priv.h>
#include <app_reset.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"


#define DIMMABLE_GPIO_0 GPIO_NUM_6
#define DIMMABLE_GPIO_1 GPIO_NUM_7

#define ONOFF_GPIO_0 GPIO_NUM_2
#define ONOFF_GPIO_1 GPIO_NUM_3

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define GPIO_INPUT_TOUCH_SENSER     GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_TOUCH_SENSER) 

#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "app_main";
uint16_t onoff_plug_endpoint_id0 = 0;
uint16_t onoff_plug_endpoint_id1 = 0;
uint16_t dimmable_plug_endpoint_id0 = 0;
uint16_t dimmable_plug_endpoint_id1 = 0;
uint16_t contact_senser_endpoint_id = 0;




using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %d, effect: %d", type, effect_id);
    return ESP_OK;
}

static void control_ledc(ledc_channel_t channel, uint32_t duty)
{
    ESP_LOGI(TAG, "control_ledc CHANNEL: %d DUTY: %d", channel, duty);
    duty = duty * 32;
    if(duty<=32)duty=0;
    if(duty>=8128)duty = 8191;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        if(endpoint_id == onoff_plug_endpoint_id0 || endpoint_id == onoff_plug_endpoint_id1){
            gpio_num_t gpio;
            if(endpoint_id == onoff_plug_endpoint_id0){
                gpio = ONOFF_GPIO_0;
            }else{
                gpio = ONOFF_GPIO_1;
            }
            if (cluster_id == OnOff::Id) {
                if (attribute_id == OnOff::Attributes::OnOff::Id) {
                    gpio_set_level(gpio, val->val.b);
                    return err;
                }
            }
        }
        else if(endpoint_id == dimmable_plug_endpoint_id0 || endpoint_id == dimmable_plug_endpoint_id1){
            ledc_channel_t channel;
            if(endpoint_id == dimmable_plug_endpoint_id0){
                channel = LEDC_CHANNEL_0;
            }else{
                channel = LEDC_CHANNEL_1;
            }
            if (cluster_id == OnOff::Id) {
                if (attribute_id == OnOff::Attributes::OnOff::Id) {
                    if(val->val.b){
                        control_ledc(channel, 255);
                    }else{
                        control_ledc(channel, 0);
                    }
                    ledc_update_duty(LEDC_MODE, channel);
                    return err;
                }
            }else if(cluster_id == LevelControl::Id){
                if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                    control_ledc(channel, val->val.b);
                }
            }
        }
        


        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

static void dimmable_plug_pwm_init(void)
{
    ESP_LOGI(TAG, "dimmable_plug_pwm_init");
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = DIMMABLE_GPIO_0,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num       = DIMMABLE_GPIO_1,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
    control_ledc(LEDC_CHANNEL_0, 0);
    control_ledc(LEDC_CHANNEL_1, 0);
}

static void onoff_plug_init(void)
{
    gpio_reset_pin(ONOFF_GPIO_0);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ONOFF_GPIO_0, GPIO_MODE_OUTPUT);
    gpio_set_level(ONOFF_GPIO_0, false);

    gpio_reset_pin(ONOFF_GPIO_1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ONOFF_GPIO_1, GPIO_MODE_OUTPUT);
    gpio_set_level(ONOFF_GPIO_1, false);
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    printf("GPIO[%d] intr, val: %d\n", gpio_num, gpio_get_level((gpio_num_t)gpio_num));
    esp_matter_attr_val_t val = esp_matter_bool(gpio_get_level((gpio_num_t)gpio_num));
    ESP_ERROR_CHECK(attribute::update(contact_senser_endpoint_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &val));
}

static void contact_sensor_init(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_TOUCH_SENSER, gpio_isr_handler, (void*) GPIO_INPUT_TOUCH_SENSER);
}

// static void periodic_timer_callback(void* arg)
// {
//     int64_t time_since_boot = esp_timer_get_time();
//     ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
//     node_t *node = node::get();
//     endpoint_t *endpoint = endpoint::get(node, light_endpoint3_id);
//     cluster_t *cluster = NULL;
//     attribute_t *attribute = NULL;
//     esp_matter_attr_val_t val = esp_matter_bool(touch_bool);
//     touch_bool = !touch_bool;
    
//     /* Setting brightness */
//     cluster = cluster::get(endpoint, BooleanState::Id);
//     attribute = attribute::get(cluster, BooleanState::Attributes::StateValue::Id);
//     ESP_ERROR_CHECK(attribute::update(light_endpoint3_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &val));
// }

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize driver */
    app_driver_handle_t light_handle = app_driver_light_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    // color_temperature_light::config_t light_config;
    // light_config.on_off.on_off = DEFAULT_POWER;
    // light_config.on_off.lighting.start_up_on_off = nullptr;
    // light_config.level_control.current_level = 1;//DEFAULT_BRIGHTNESS;
    // light_config.level_control.lighting.start_up_current_level = 1;//DEFAULT_BRIGHTNESS;
    // light_config.color_control.color_mode = EMBER_ZCL_COLOR_MODE_COLOR_TEMPERATURE;
    // light_config.color_control.enhanced_color_mode = EMBER_ZCL_COLOR_MODE_COLOR_TEMPERATURE;
    // light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;
    // endpoint_t *endpoint = color_temperature_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);

    // /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
    // if (!node || !endpoint) {
    //     ESP_LOGE(TAG, "Matter node creation failed");
    // }

    // light_endpoint_id = endpoint::get_id(endpoint);
    // ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

    // /* Add additional features to the node */
    // cluster_t *cluster = cluster::get(endpoint, ColorControl::Id);
    // cluster::color_control::feature::hue_saturation::config_t hue_saturation_config;
    // hue_saturation_config.current_hue = DEFAULT_HUE;
    // hue_saturation_config.current_saturation = DEFAULT_SATURATION;
    // cluster::color_control::feature::hue_saturation::add(cluster, &hue_saturation_config);

    // contact_sensor::config_t contact_senser_config;
    // contact_senser_config.boolean_state.state_value = false;
    // endpoint_t *endpoint3 = contact_sensor::create(node, &contact_senser_config, ENDPOINT_FLAG_NONE, 0);

    // if(!endpoint3){
    //     ESP_LOGE(TAG, "Test contact_sensor creation failed");
    // }
    // light_endpoint3_id = endpoint::get_id(endpoint3);
    // ESP_LOGI(TAG, "Test contact_sensor created with endpoint_id %d", light_endpoint3_id);

    on_off_plugin_unit::config_t plugin_config0;
    plugin_config0.on_off.on_off = false;
    endpoint_t *onoff_plugin_endpoint0 = on_off_plugin_unit::create(node, &plugin_config0, ENDPOINT_FLAG_NONE, 0);
    onoff_plug_endpoint_id0 = endpoint::get_id(onoff_plugin_endpoint0);
    ESP_LOGI(TAG, "on_off_plugin0 created with endpoint_id %d", onoff_plug_endpoint_id0);

    on_off_plugin_unit::config_t plugin_config1;
    plugin_config1.on_off.on_off = false;
    endpoint_t *onoff_plugin_endpoint1 = on_off_plugin_unit::create(node, &plugin_config0, ENDPOINT_FLAG_NONE, 0);
    onoff_plug_endpoint_id1 = endpoint::get_id(onoff_plugin_endpoint1);
    ESP_LOGI(TAG, "on_off_plugin1 created with endpoint_id %d", onoff_plug_endpoint_id1);



    dimmable_plugin_unit::config_t dimmable_plugin_config0;
    dimmable_plugin_config0.on_off.on_off = false;
    dimmable_plugin_config0.level_control.current_level = (uint8_t)0;
    endpoint_t *dimmable_plugin_endpoint0 = dimmable_plugin_unit::create(node, &dimmable_plugin_config0, ENDPOINT_FLAG_NONE, 0);
    dimmable_plug_endpoint_id0 = endpoint::get_id(dimmable_plugin_endpoint0);
    ESP_LOGI(TAG, "dimmable_plugin0 created with endpoint_id %d", dimmable_plug_endpoint_id0);

    dimmable_plugin_unit::config_t dimmable_plugin_config1;
    dimmable_plugin_config1.on_off.on_off = false;
    dimmable_plugin_config1.level_control.current_level = (uint8_t)0;
    endpoint_t *dimmable_plugin_endpoint1 = dimmable_plugin_unit::create(node, &dimmable_plugin_config1, ENDPOINT_FLAG_NONE, 0);
    dimmable_plug_endpoint_id1 = endpoint::get_id(dimmable_plugin_endpoint1);
    ESP_LOGI(TAG, "dimmable_plugin1 created with endpoint_id %d", dimmable_plug_endpoint_id1);

    dimmable_plug_pwm_init();
    onoff_plug_init();
    contact_sensor_init();


    /* Matter start */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
    }

    /* Starting driver with default values */
    // app_driver_light_set_defaults(light_endpoint_id);



#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::init();
#endif
}
