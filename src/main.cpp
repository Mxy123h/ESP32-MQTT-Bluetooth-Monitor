// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <initializer_list>
// IDF headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include "time.h"

#include <ezTime.h>

#include "esp_task_wdt.h"

// Custom components:
#include "NestedStruct.h"
#include "TelnetSerial.h"
#include "WiFiComponent.h"
#include "BluetoothScanner.h"
#include "parameter.h"
#include "BluetoothParameter.h"
#include "led.h"
#include "mqtt.h"

#include "stackDbgHelper.h"

#ifndef DDUMPTIMEMEMINTERVAL
#define DDUMPTIMEMEMINTERVAL 10
#endif

#ifndef MAX_NUM_STORED_BLUETOOTH_DEVICES
#define MAX_NUM_STORED_BLUETOOTH_DEVICES (8)
#endif
// Upper limit for max number of stored bluetooth devices. 
// Is rediculously large, but more would not fit in NVS!!
static_assert(MAX_NUM_STORED_BLUETOOTH_DEVICES <= 40, "Cannot fit more than 40 bluetooth device identifiers in ESP32 storage!!");

#define WDT_TIMEOUT 8

// BOOT 键默认连接到 GPIO0，按键按下时由内部上拉读取为低电平
#ifndef BOOT_BUTTON_GPIO
#define BOOT_BUTTON_GPIO 0
#endif

#ifndef BOOT_BUTTON_LONG_PRESS_MS
#define BOOT_BUTTON_LONG_PRESS_MS 5000
#endif

#ifndef BOOT_BUTTON_SAMPLE_INTERVAL_MS
#define BOOT_BUTTON_SAMPLE_INTERVAL_MS 20
#endif

// ================================================
// >>>>>>>>>>  Global objects  >>>>>>>>>>>>>>>>>>>>

#ifndef NO_USE_TELNET_SERIAL
TelnetSerial telnetSerial;
Stream& mSerial = telnetSerial;
#else
Stream& mSerial = Serial;
#endif
Preferences preferences;
WiFiManager wm(mSerial);
LED led;

// >>> All parameters in order shown in WiFiManager:
Parameter time_header   ("<h3>时区</h3><br>例如 Asia/Shanghai。<a href='http://wikipedia.org/wiki/List_of_tz_database_time_zones#list'>时区列表</a><br>留空则自动定位。<br>");
Parameter time_zone     ("time_zone",     "",            "",         40);
// MQTT 
Parameter mqtt_header   ("<h3>MQTT 参数</h3>");
Parameter mqtt_server   ("mqtt_server",   "MQTT 服务器", "",         40);
Parameter mqtt_port     ("mqtt_port",     "MQTT 端口",   "1883",      6);
Parameter mqtt_username ("mqtt_username", "用户名",      "",         32);
Parameter mqtt_password ("mqtt_password", "密码",        "",         32);
Parameter mqtt_client_id("mqtt_clid",     "客户端 ID",   "",         32);

Parameter mqtt_topic   ("mqtt_topic",    "主题根路径",  "monitor",  32);
Parameter mqtt_identity("mqtt_identity", "设备标识",    "",         32);

// Bluetooth Monitor settings
Parameter     bluetooth_monitor_header                     (PSTR("<h3>蓝牙监视器设置</h3>"));
 U16Parameter  bluetooth_monitor_arrival_scans              (PSTR("bm_arrival"),    PSTR("到达扫描次数"),                 1  );
 U16Parameter  bluetooth_monitor_departure_scans            (PSTR("bm_depart"),     PSTR("离开扫描次数"),                 3  );
 U16Parameter  bluetooth_monitor_seconds_between_scan_iters (PSTR("bm_iter_time"),  PSTR("扫描间隔（秒）"),               3  );
 U16Parameter  bluetooth_monitor_scan_timeout_seconds       (PSTR("bm_timeout"),    PSTR("扫描超时（秒）"),               60 );
 U16Parameter  bluetooth_monitor_beacon_expiration          (PSTR("bm_beacon_exp"), PSTR("信标过期时间（秒）"),           240);
 U16Parameter  bluetooth_monitor_min_time_between_scans     (PSTR("bm_min_time"),   PSTR("两轮扫描最小间隔（秒）"),       10 );
 U16Parameter  bluetooth_monitor_periodic_scan_interval     (PSTR("bm_period"),     PSTR("定期扫描间隔（秒）<br>留空或填 0 可关闭定期扫描"), 0);
 BoolParameter bluetooth_monitor_retain_flag                (PSTR("bm_retain"),     PSTR("MQTT 保留消息（true/false）"), false);

// Known Static Bluetooth MAC 
// Nested struct helper to generate objects and reduce boilerplate...
Parameter bluetooth_monitor_devices_header (PSTR("<h3>蓝牙监视设备</h3>"));
NestWrapper<BluetoothParameter, MAX_NUM_STORED_BLUETOOTH_DEVICES> bluetooth_monitor_parameter_sets;

// <<<
WiFiComponent wifi(mSerial);
BluetoothScanner btScanner(mSerial);
MQTT mqtt(mSerial);

Timezone mTime;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


std::stack<stack_entry> _stack;

uint32_t startFreeHeap = 0;

#define USE_SEPARATE_STATS_TASK 0

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_task_info.h"

#define MAX_TASK_NUM 20                         // Max number of per tasks info that it can store
#define MAX_BLOCK_NUM 20                        // Max number of per block info that it can store

static size_t s_prepopulated_num = 0;
static heap_task_totals_t s_totals_arr[MAX_TASK_NUM];
static heap_task_block_t s_block_arr[MAX_BLOCK_NUM];

// -----------------------------------------------
extern "C" void esp_task_wdt_isr_user_handler(void)
{
    printf("\nSTACK:\n");
    while(!_stack.empty()) {
        auto& e = _stack.top();
        printf("PC: %d, Func: %s, Line: %d \n", e.pc, e.func, e.line);
        _stack.pop();
    }
}

// -----------------------------------------------
static void esp_dump_per_task_heap_info(void)
{
    SCOPED_STACK_ENTRY;
    heap_task_info_params_t heap_info = {0};
    heap_info.caps[0] = MALLOC_CAP_8BIT;        // Gets heap with CAP_8BIT capabilities
    heap_info.mask[0] = MALLOC_CAP_8BIT;
    heap_info.caps[1] = MALLOC_CAP_32BIT;       // Gets heap info with CAP_32BIT capabilities
    heap_info.mask[1] = MALLOC_CAP_32BIT;
    heap_info.tasks = NULL;                     // Passing NULL captures heap info for all tasks
    heap_info.num_tasks = 0;
    heap_info.totals = s_totals_arr;            // Gets task wise allocation details
    heap_info.num_totals = &s_prepopulated_num;
    heap_info.max_totals = MAX_TASK_NUM;        // Maximum length of "s_totals_arr"
    heap_info.blocks = s_block_arr;             // Gets block wise allocation details. For each block, gets owner task, address and size
    heap_info.max_blocks = MAX_BLOCK_NUM;       // Maximum length of "s_block_arr"

    heap_caps_get_per_task_info(&heap_info);

    for (int i = 0 ; i < *heap_info.num_totals; i++) {
        const char* taskName = heap_info.totals[i].task ? pcTaskGetTaskName(heap_info.totals[i].task) : nullptr;
        bool tryHeap = true;
        if(taskName == nullptr) {
            tryHeap = false;
        }
        else {
            for (int i = 0; i < strlen(taskName); i++) {
                if(!isascii(taskName[i])) {
                    tryHeap = false;
                    break;
                }
            }
        }
        
        mSerial.printf("Task: %22s -> CAP_8BIT: %6d CAP_32BIT: %3d -> HighWaterMark: %5d\n",
                heap_info.totals[i].task ? pcTaskGetTaskName(heap_info.totals[i].task) : "Pre-Scheduler allocs" ,
                heap_info.totals[i].size[0],    // Heap size with CAP_8BIT capabilities
                heap_info.totals[i].size[1],    // Heap size with CAP32_BIT capabilities
                tryHeap ? uxTaskGetStackHighWaterMark(heap_info.totals[i].task) : -1);   
                
        feedLoopWDT(); // Must be done <= 360 mS
    }

    mSerial.printf("Free heap at start: %d, min free heap ever: %d\n", startFreeHeap, xPortGetMinimumEverFreeHeapSize());
    mSerial.printf("Free heap size: %d, largest fee block: %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    mSerial.print("HeapCapsFreeSize - Internal: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    mSerial.print(" SPIRAM: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    mSerial.print(" IRAM8: ");
    mSerial.print(heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT));
    mSerial.print(" Default: ");
    mSerial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    mSerial.printf("\n\n");
}

// -----------------------------------------------
void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
    // No Printf to avoid MALLOCS!!!
    Serial.print(function_name);
    Serial.print(" in task ");
    Serial.print(pcTaskGetTaskName(NULL));
    Serial.print(" was called but failed to allocate ");
    Serial.print(requested_size);
    Serial.print(" bytes, with capabilities: ");
    Serial.println(caps);
    Serial.print(" .High water mark: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    Serial.print("HeapCapsFreeSize - Internal: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.print(" SPIRAM: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.print(" IRAM8: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT));
    Serial.print(" Default: ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    
    Serial.print("Free heap at start: ");
    Serial.print(startFreeHeap);
    Serial.print(", min free heap ever: ");
    Serial.println(xPortGetMinimumEverFreeHeapSize());
    Serial.print("Free heap size: ");
    Serial.print(heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.print(", largest fee block: ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    //Serial.printf("%s in task '%s' was called but failed to allocate %d bytes with 0x%X capabilities. HighWaterMark: %lu \n",function_name, pcTaskGetTaskName(NULL), requested_size, caps, (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

// -----------------------------------------------
bool ensureAllCaps(std::string& str) {
    bool strUpdated = false;
    for (auto & c: str) {
        auto cu = toupper(c);
        if(c != cu) {
            strUpdated = true;
        }
        c = cu;
    }
    return strUpdated;
}

// -----------------------------------------------
// 后台监测 BOOT 长按，主循环被 WiFi 配网页面阻塞时仍能执行恢复
void bootButtonResetTask(void*) {
    pinMode(BOOT_BUTTON_GPIO, INPUT_PULLUP);
    uint32_t pressStartedAt = 0;
    uint32_t nextProgressAt = 0;

    for (;;) {
        const uint32_t now = millis();
        if (digitalRead(BOOT_BUTTON_GPIO) == LOW) {
            if (pressStartedAt == 0) {
                pressStartedAt = now;
                nextProgressAt = now;
                Serial.printf("检测到 BOOT 按键，请持续按住 %u 秒以清除全部配置...\n", BOOT_BUTTON_LONG_PRESS_MS / 1000);
            }

            const uint32_t elapsed = now - pressStartedAt;
            if (elapsed >= BOOT_BUTTON_LONG_PRESS_MS) {
                Serial.println("BOOT 长按确认，正在清除 NVS 配置...");
                preferences.end();
                const esp_err_t eraseError = nvs_flash_erase();
                const esp_err_t initError = nvs_flash_init();
                Serial.printf("NVS 清除结果 erase=%d init=%d\n", eraseError, initError);
                Serial.println("请松开 BOOT 按键，设备即将重启");

                // 必须先等按键释放，避免重启后 GPIO0 仍为低电平而进入下载模式
                while (digitalRead(BOOT_BUTTON_GPIO) == LOW) {
                    delay(BOOT_BUTTON_SAMPLE_INTERVAL_MS);
                }
                delay(100);
                ESP.restart();
                vTaskDelete(nullptr);
            }

            if (now >= nextProgressAt) {
                Serial.printf("BOOT 长按进度 %lu/%u ms\n", static_cast<unsigned long>(elapsed), BOOT_BUTTON_LONG_PRESS_MS);
                nextProgressAt += 1000;
            }
        }
        else {
            if (pressStartedAt != 0) {
                Serial.println("BOOT 按键已释放，未执行清除");
            }
            pressStartedAt = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(BOOT_BUTTON_SAMPLE_INTERVAL_MS));
    }
}

// -----------------------------------------------
#define PREFERENCES_TAG_VALUE (54)
void setupPreferences() {
    preferences.begin("E32BT", false);
    
    // Perform a factory reset if we have not ran before...
    if(preferences.getUInt("ESP32_BT_TAG", 0) != PREFERENCES_TAG_VALUE) {
        Serial.print("Could not find Magic value in preferences. Clearing NVS for factory reset!");

        preferences.end();

        nvs_flash_erase(); // erase the NVS partition and...
        nvs_flash_init(); // initialize the NVS partition.
        Serial.print(" ... ");

        preferences.begin("E32BT", false);
        preferences.putUInt("ESP32_BT_TAG", PREFERENCES_TAG_VALUE);
        Serial.println("Done!!");
    }

    // 参数页稍后才会从 NVS 加载。这里必须直接读取持久化值，
    // 否则每次启动都可能先把内存中的客户端 ID 误设为默认值。
    String client_id = preferences.getString("mqtt_clid", "");
    if(client_id.length() < 1) {
        client_id = String("ESP32_bt") + String(WIFI_getChipId(),HEX);
        preferences.putString("mqtt_clid", client_id);
    }
    mqtt_client_id.setValue(client_id.c_str());

    // Remove all preferences under the opened namespace
    //preferences.clear();

    // Or remove the counter key only
    //preferences.remove("counter");
}

// -----------------------------------------------
void setupMQTT() {
    mqtt.setup( mqtt_server.getValue(),
                mqtt_port.getValue(),
                mqtt_username.getValue(),
                mqtt_password.getValue(),
                mqtt_client_id.getValue() );
}

// -----------------------------------------------
void setupMqttCallbacks() {
    // Clear existing topics and callbacks based on old parameters:
    mqtt.clear_subscription_topics();
    mqtt.clear_callbacks();

    std::string baseTopic = mqtt.trimWildcards(mqtt_topic.getValue());
    // 在线状态直接发布到扫描器主题根，不再追加 /status。
    mqtt.setStateTopic(baseTopic + "/" + mqtt_identity.getValue());
    mqtt.setIpTopic(baseTopic + "/" + mqtt_identity.getValue() + "/IP");

    // ToDo: this becomes a problem when reloading parameters at runtime!!
    // FIX THIS!
    //mqtt.add_subscription_topic(mqtt_topic.getValue()); //TEST
    mqtt.add_subscription_topic((baseTopic + "/scan/#").c_str());
    mqtt.add_subscription_topic((baseTopic + "/setup/#").c_str());
    mqtt.add_subscription_topic((baseTopic + "/" + mqtt_identity.getValue() + "/restart").c_str());
    mqtt.add_callback((baseTopic + "/" + mqtt_identity.getValue() + "/restart").c_str(), [](const byte* payload, unsigned int length) {
        mSerial.println("Restarting!!");
        delay(1000);
        ESP.restart();
    });
    mqtt.add_callback((baseTopic + "/scan/ARRIVE").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Arrival);
    });
    mqtt.add_callback((baseTopic + "/scan/DEPART").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Departure);
    });
    mqtt.add_callback((baseTopic + "/scan/ANY").c_str(), [](const byte* payload, unsigned int length) {
        btScanner.startBluetoothScan(ScanType::Either);
    });
    // MQTT setup interfaces:
    mqtt.add_callback((baseTopic + "/setup/ADD STATIC DEVICE").c_str(), [](const byte* payload, unsigned int length) {
        // Splitting the MAC from the alias:
        std::string _in = std::string((const char*)payload, length);
        _in.erase(_in.find_last_not_of(" \t\n\v") + 1);
        std::string mac = _in.substr(0, _in.find(" "));
        std::string alias = _in.substr(_in.find(" "));

        size_t first = alias.find_first_not_of(" \t\n\v");
        size_t last = alias.find_last_not_of(" \t\n\v");
        if(first != std::string::npos) {
            alias = alias.substr(first, (last-first+1));
        }

        BLEUUID ble_uuid = BLEUUID::fromString(mac);

        // Add to storage and BluetoothScanner
        esp_bd_addr_t addr;
        if(str2bda(mac.c_str(), addr)) {
            char bda_str[18];
            if (bda2str(addr, bda_str, 18)) {
                bool device_saved = false;
                for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
                    if(strlen(bluetooth_monitor_parameter_sets.data[i].getMacAddress()) == 0 || strcmp(bluetooth_monitor_parameter_sets.data[i].getMacAddress(), bda_str) == 0) {
                        bluetooth_monitor_parameter_sets.data[i].setMacAddress(bda_str);
                        bluetooth_monitor_parameter_sets.data[i].setAlias(alias.c_str());
                        device_saved = true;
                        break;
                    }
                }
                if (device_saved){
                    btScanner.addKnownDevice(addr, alias.c_str());
                }
            }
        }
        else if (!ble_uuid.equals(BLEUUID())) {
            bool device_saved = false;
            for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
                if(strlen(bluetooth_monitor_parameter_sets.data[i].getMacAddress()) == 0 || strcmp(bluetooth_monitor_parameter_sets.data[i].getMacAddress(), ble_uuid.toString().c_str()) == 0) {
                    bluetooth_monitor_parameter_sets.data[i].setMacAddress(ble_uuid.toString().c_str());
                    bluetooth_monitor_parameter_sets.data[i].setAlias(alias.c_str());
                    device_saved = true;
                    break;
                }
            }
            if (device_saved){
                btScanner.addKnownIBeacon(ble_uuid, alias.c_str());
            }
        }

    });
    mqtt.add_callback((baseTopic + "/setup/DELETE STATIC DEVICE").c_str(), [](const byte* payload, unsigned int length) {
        esp_bd_addr_t addr;
        if(str2bda(std::string((const char*)payload, length).c_str(), addr)) {
            char bda_str[18];
            if (bda2str(addr, bda_str, 18)) {
                btScanner.deleteKnownDevice(addr);
                for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
                    if(strcmp(bluetooth_monitor_parameter_sets.data[i].getMacAddress(), bda_str) == 0) {
                        bluetooth_monitor_parameter_sets.data[i].setMacAddress("");
                        bluetooth_monitor_parameter_sets.data[i].setAlias("");
                    }
                }
            }
        }
    });
}

// -----------------------------------------------
void setupBtScanner() {    
    btScanner.setMqttTopic(mqtt.trimWildcards(mqtt_topic.getValue()));
    btScanner.setScannerIdentity(mqtt_identity.getValue());
    btScanner.setRetainFlag(param2bool(bluetooth_monitor_retain_flag));        

    btScanner.setNumArrivalScans        (bluetooth_monitor_arrival_scans.getValue());
    btScanner.setNumDepartureScans      (bluetooth_monitor_departure_scans.getValue());
    btScanner.setSecondsBetweenScanIters(bluetooth_monitor_seconds_between_scan_iters.getValue());
    btScanner.setBeaconExpiration       (bluetooth_monitor_beacon_expiration.getValue());
    btScanner.setMinTimeBetweenScans    (bluetooth_monitor_min_time_between_scans.getValue());
    btScanner.setPeriodicScanInterval   (bluetooth_monitor_periodic_scan_interval.getValue());
    btScanner.setScanDurationTimeout    (bluetooth_monitor_scan_timeout_seconds.getValue());
    // ToDo: Reload the rest of Bluetooth parameters

    // (Re)Load known BT devices:
    btScanner.clearKnownDevices();
    for(int i = 0; i < bluetooth_monitor_parameter_sets.size; i++) {
        esp_bd_addr_t mac;
        std::string mac_str = bluetooth_monitor_parameter_sets.data[i].getMacAddress();

        // Ensure Mac addresses are all CAPS:
        bool updateMac = ensureAllCaps(mac_str);
        if(updateMac) {
            bluetooth_monitor_parameter_sets.data[i].setMacAddress(mac_str.c_str());
        }

        if(mac_str.find_first_not_of(" \t\n\v\f\r") != std::string::npos) {
            mSerial.printf("Validating MAC: '%s' \n", mac_str.c_str());

            BLEUUID ble_uuid = BLEUUID::fromString(mac_str);

            // If valid mac address, add it!
            if(str2bda(mac_str.c_str(), mac)) {
                const char* alias = bluetooth_monitor_parameter_sets.data[i].getAlias();
                mSerial.printf("  Adding device: %s, with MAC: %s \n", alias, mac_str.c_str());
                btScanner.addKnownDevice(mac, alias);
            }
            else if (!ble_uuid.equals(BLEUUID())) {
                const char* alias = bluetooth_monitor_parameter_sets.data[i].getAlias();
                mSerial.printf("  Adding device: %s, with UUID: '%s' \n", alias, mac_str.c_str());
                btScanner.addKnownIBeacon(ble_uuid, alias);
            }
        }
    }

}

// -----------------------------------------------
void setupTimeZone() {    
	// Or country codes for countries that do not span multiple timezones
	//mTime.setLocation(F("nl"));
    const char* timezone = time_zone.getValue();
	mTime.setLocation(strlen(timezone) ? timezone : "GeoIP");
	Serial.print(F("Local time:         "));
	Serial.println(mTime.dateTime());
}


#if CONFIG_AUTOSTART_ARDUINO
// -----------------------------------------------
void setup() {
    startFreeHeap = xPortGetFreeHeapSize();
    esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

    // Turn on LED at start of setup
    led.setup();
    led.set(ON);
     
    WiFi.setAutoConnect(true);

    // Begin Serial separate!
    Serial.begin(115200, SERIAL_8N1);

    const BaseType_t bootTaskResult = xTaskCreatePinnedToCore(bootButtonResetTask, "boot_button", 3072, nullptr, 1, nullptr, 0);
    if (bootTaskResult != pdPASS) {
        Serial.println("BOOT 按键监测任务创建失败");
    }
    setupPreferences();

    mSerial.println("Booting");
    ezt::setDebug(INFO, mSerial);

    if(error != ESP_OK) {
        Serial.printf("failed alloc callback error: %d\n", error);
    }

    // Note that this will not show up in the MENU!!!
    wifi.addCustomHtmlPage("/bt", [](){
        String page = FPSTR(HTTP_HEAD_START);
        page.replace(FPSTR(T_v), "蓝牙状态");
        page += FPSTR(HTTP_SCRIPT);
        page += FPSTR(HTTP_STYLE);
        // Selection for DarkMode: _bodyClass = enable ? "invert" : "";
        String p = FPSTR(HTTP_HEAD_END);
        p.replace(FPSTR(T_c), "invert"); // add class str
        page += p;

        auto& devList = btScanner.getBtDeviceStates();
        char bda_str[18];
        for (auto& dev : devList) {
            String str = FPSTR("<div class='msg {C}'><strong>{n}</strong> is {s}<br/><em><small>{m}</small></em></div>");
            str.replace("{C}", (dev.state == 1 ? "S" : "D"));
            str.replace("{n}", dev.name);
            str.replace("{s}", (dev.state == 1 ? "在范围内" : "已离开"));
            bda2str(dev.mac, bda_str, 18);
            str.replace("{m}", bda_str);
            page += str;
        }

        page += FPSTR("<hr><br/>"  // MENU_SEP
                      "<br/><form action='/bt?arrive=1' method='POST'><button name='arrival' value='1'>到达扫描</button></form>"
                      "<br/><form action='/bt?depart=1' method='POST'><button name='departure' value='1'>离开扫描</button></form>"
                      "<br/><form action='/bt?scan=1' method='POST'><button name='scan' value='1'>全量扫描</button></form>"
                      "<br/><form action='/bt?refresh=1' method='POST'><button name='refresh' value='1'>刷新</button></form>");

        if(wm.server->hasArg(F("arrive"))) {
            btScanner.startBluetoothScan(ScanType::Arrival);
        }
        if(wm.server->hasArg(F("depart"))) {
            btScanner.startBluetoothScan(ScanType::Departure);
        }
        if(wm.server->hasArg(F("scan"))) {
            btScanner.startBluetoothScan(ScanType::Either);
        }

        page += FPSTR(HTTP_END);

        wm.server->send(200, FPSTR(HTTP_HEAD_CT), page);        
    });

    // Blocking if not connected WiFi AP
    wifi.setup();
    // Stop bluetooth on OTA to reduce chance of failures..
    wifi.registerOtaStartCallback([](){
        mSerial.println("Stopping BtScanner for OTA...");
        btScanner.stop();
    });

#ifndef NO_USE_TELNET_SERIAL
    telnetSerial.begin(); // Start the telnet bit...
#endif
    setupMQTT();
    setupMqttCallbacks();

    // Setup bluetooth AFTER WiFi such that the persistent parameters are loaded!
    btScanner.init();  // ESP Bluetooth component, HAL, etc.
    setupBtScanner(); // Scanner configuration / settings 

    // EZTime: sync time
	ezt::waitForSync();
    setupTimeZone();

    // Stop bluetooth on OTA to reduce chance of failures..
    wifi.registerOtaStartCallback([](){
        mSerial.println("Stopping BtScanner for OTA...");
        btScanner.stop();
    });
    // Run MQTT setup again when new parameters are made available
    wifi.registerParamSaveCallback(setupMQTT); // This causes pubSub to destruct and panic!!
    wifi.registerParamSaveCallback(setupMqttCallbacks);
    wifi.registerParamSaveCallback(setupTimeZone);
    wifi.registerParamSaveCallback(setupBtScanner);

    // Turn off led at the end of setup
    led.set(OFF);

    btScanner.startBluetoothScan(ScanType::Either);

    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts

    enableLoopWDT();
}

uint16_t count = 0;

unsigned long last_millis = 0;
// unsigned long interval=10000; // 10sec
unsigned long interval=DUMPTIMEMEMINTERVAL*1000; // 10sec

void loop() {
    SCOPED_STACK_ENTRY;

    wifi.loop();
#ifndef NO_USE_TELNET_SERIAL
    telnetSerial.loop();
#endif

    mqtt.loop();
    delay(10);
    btScanner.loop();

    unsigned long current_millis = millis();
    if(interval > 0 && current_millis > last_millis + interval) {
        last_millis = current_millis;
        mSerial.println(current_millis);
        
	    mSerial.println(mTime.dateTime());
        esp_dump_per_task_heap_info();
        
        count++;
    }    

    // WDT Feeding is done in Arduino scope outside of the loop...
	// Delay needed to give idle task some room!!
    delay(10);
}
#else
#ERROR wrong SDK configuration!
#endif 
