
// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>
#include "esp_sntp.h"

#include "parameter.h"

#include "stackDbgHelper.h"
#include "WiFiComponent.h"
#include "led.h"

// Required available global entities:
extern WiFiManager wm;

const char* worldNtpServer = "pool.ntp.org";
const char* eurNtpServer   = "europe.pool.ntp.org";
const char* nlNtpServer    = "nl.pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// WiFiManager 当前依赖仅提供英文内置文案；通过页头脚本统一替换，避免修改第三方库。
const char CHINESE_HEAD_ELEMENT[] = R"rawliteral(
<script>
document.addEventListener('DOMContentLoaded', function () {
  const textMap = {
    'Configure WiFi (No Scan)': '配置 WiFi（不扫描）',
    'Configure WiFi': '配置 WiFi',
    'Erase WiFi Config': '清除 WiFi 配置',
    'Saving Credentials': '正在保存 WiFi 凭据',
    'Trying to connect ESP to network.': '正在连接到 WiFi 网络。',
    'If it fails reconnect to AP to try again': '如连接失败，请重新连接热点后再试。',
    'No networks found. Refresh to scan again.': '未找到 WiFi 网络，请刷新后重新扫描。',
    'Show Password': '显示密码',
    'Access Point Hostname': '热点主机名',
    'Access Point IP': '热点 IP 地址',
    'Access Point MAC': '热点 MAC 地址',
    'Access Point SSID': '热点名称',
    'Station Gateway': 'STA 网关',
    'Station Subnet': 'STA 子网掩码',
    'Station SSID': '已连接 WiFi',
    'Station IP': 'STA IP 地址',
    'Station MAC': 'STA MAC 地址',
    'DNS Server': 'DNS 服务器',
    'CPU Frequency': 'CPU 频率',
    'SDK Version': 'SDK 版本',
    'Flash Size': '闪存大小',
    'Chip Rev': '芯片修订版本',
    'Last reset reason': '上次复位原因',
    'Memory - Sketch Size': '程序存储空间',
    'Memory - Free Heap': '可用堆内存',
    'Upload New Firmware': '上传新固件',
    'Update Successful.': '固件更新成功。',
    'Update Failed!': '固件更新失败！',
    'Device Rebooting now...': '设备正在重启……',
    'Restart': '重启设备',
    'Refresh': '刷新',
    'Settings Saved': '设置已保存',
    'Setup Saved': '设置已保存',
    'Credentials Saved': 'WiFi 凭据已保存',
    'Config ESP': 'ESP 配网',
    'No AP set': '未设置热点',
    'Reboot device and try again': '请重启设备后重试。',
    'Used / Total bytes': '已用 / 总字节',
    'bytes available': '字节可用',
    'with IP': 'IP 地址',
    'Connected': '已连接',
    'Not Connected': '未连接',
    'Password': '密码',
    'Save': '保存',
    'Setup': '设置',
    'Info': '设备信息',
    'Erase': '清除配置',
    'Update': '固件更新',
    'Back': '返回'
  };
  const nodes = [];
  const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  while (walker.nextNode()) nodes.push(walker.currentNode);
  nodes.forEach(function (node) {
    let value = node.nodeValue;
    Object.keys(textMap).forEach(function (english) {
      value = value.split(english).join(textMap[english]);
    });
    node.nodeValue = value;
  });
  document.title = 'ESP32 蓝牙监视器';
});
</script>
)rawliteral";

// -----------------------------------------------
void WiFiComponent::wifiInfo(){
    WiFi.printDiag(mSerial);
    mSerial.println("SAVED: " + (String)wm.getWiFiIsSaved() ? "YES" : "NO");
    mSerial.println("SSID: " + (String)wm.getWiFiSSID());
    mSerial.println("PASS: " + (String)wm.getWiFiPass());
}

// -----------------------------------------------
void WiFiComponent::setup() {
    // Enable AP mode as well as STA, such that WM can create an access point if needed
    WiFi.mode(WIFI_MODE_APSTA);

    _hostname = String("ESP32_bt") + String(WIFI_getChipId(),HEX);
    
    // Load registered parameters from NVS
    loadParams();

    // Blocking if not connected WiFi AP
    setupConfigPortal();

    _wifiWasConnected = WiFi.status() == WL_CONNECTED;

    setupArduinoOTA();
}

// -----------------------------------------------
void WiFiComponent::loop() {
    SCOPED_STACK_ENTRY;
    do {
        ArduinoOTA.handle();
        feedLoopWDT();
        delay(10);
    } while(otaBusy);
    wm.process();

    const unsigned long now = millis();
    const bool wifiConnected = WiFi.status() == WL_CONNECTED;

    if (wifiConnected) {
        if (!_wifiWasConnected) {
            mSerial.printf("WiFi restored, IP: %s\n", WiFi.localIP().toString().c_str());
        }
        _wifiWasConnected = true;
        _wifiReconnectPending = false;
        _lastWiFiReconnectAttempt = 0;
        return;
    }

    if (_wifiWasConnected) {
        mSerial.printf("WiFi lost, status=%d\n", WiFi.status());
        _wifiWasConnected = false;
    }

    // 不在 WiFi 事件回调中操作网络；在主循环中限频重连，避免路由器重启期间的重连竞态。
    if (_wifiReconnectPending) {
        if (now - _wifiReconnectStartedAt >= 500) {
            mSerial.println("WiFi reconnecting with saved credentials...");
            WiFi.begin();
            _wifiReconnectPending = false;
            _lastWiFiReconnectAttempt = now;
        }
    }
    else if (now - _lastWiFiReconnectAttempt >= 10000) {
        mSerial.println("WiFi disconnected, scheduling reconnect...");
        WiFi.disconnect(false, false);
        _wifiReconnectPending = true;
        _wifiReconnectStartedAt = now;
    }
}

// -----------------------------------------------
void WiFiComponent::registerOtaStartCallback(std::function<void(void)> callback) {
    ota_callback_functions.push_back(callback);
}

// -----------------------------------------------
void WiFiComponent::registerParamSaveCallback(std::function<void(void)> callback) {
    paramSaved_callback_functions.push_back(callback);
}

// -----------------------------------------------
void WiFiComponent::setupConfigPortal() {
      
    wm.debugPlatformInfo();

    //reset settings - for testing
    // wm.resetSettings();
    // wm.erase();  

    // callbacks
    wm.setAPCallback(std::bind(&WiFiComponent::configModeCallback,this, std::placeholders::_1));
    wm.setWebServerCallback(std::bind(&WiFiComponent::bindServerCallback,this));
    wm.setSaveConfigCallback(std::bind(&WiFiComponent::saveWifiCallback,this));
    wm.setSaveParamsCallback(std::bind(&WiFiComponent::saveParamCallback,this));

    wm.setTitle("ESP32 蓝牙监视器");
    wm.setHostname(_hostname.c_str());
    wm.setCustomHeadElement(CHINESE_HEAD_ELEMENT);

    // invert theme, dark
    wm.setDarkMode(true);

    // Show update button on info page
    wm.setShowInfoUpdate(true); 

    // Separate ParamsPage from WiFi page
    wm.setParamsPage(true);
        
    // Clear the original menu list
    wm.setMenu(nullptr, 0);
    // Use custom menu list only:
    wm.setCustomMenuItems({{"/wifi"   , "配置 WiFi",     false},
                           {"/param"  , "应用设置",      false},
                           {"/info"   , "设备信息",      false},
                           {"/bt"     , "蓝牙监视器",    false},
                           {"--"      , ""              , false}, // Separator
                           {"/erase"  , "清除配置",      true },
                           {"/update" , "固件更新",      false},
                           {"/restart", "重启设备",      false},
                           {"--"      , ""              , false}}); // Ending Separator


    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep in seconds
    wm.setConfigPortalTimeout(600); // 10 minutes

    // set connection timeout
    wm.setConnectTimeout(3);

    // set wifi connect retries
    wm.setConnectRetries(3);

    // connect after portal save toggle
    wm.setSaveConnect(true); // do not connect, only save

    wm.setWiFiAutoReconnect(true);

    String apName = String(WIFI_getChipId(),HEX);
    apName.toUpperCase();
    apName = "ESP32_bt_" + apName;

    // Setup httpd authentication for config portal once it's connected to users home WiFi network:
    wm.setHttpdAuthCredentials(HTTPD_USER, HTTPD_PASSWD);

    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //and goes into a blocking loop awaiting configuration
    // --> last parameter ensures a retry to the blocking loop to connect to known WiFi AP
    if(!wm.autoConnect(apName.c_str(), AP_PASSWD, true)) {
        mSerial.println("failed to connect and hit timeout");
    }
    else {
        wm.setHttpdAuthEnable(true); 

        // Disable AP
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);

        //if you get here you have connected to the WiFi
        mSerial.println("connected...yeey :)");
        wm.startWebPortal();
    }

    wifiInfo();
}

// -----------------------------------------------
void WiFiComponent::setupArduinoOTA() {
    
    ArduinoOTA.setPort(3232);

    ArduinoOTA.setHostname("ESP32_BT");

    ArduinoOTA.setPassword("admin");

    ArduinoOTA
    .onStart([this]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
        else // U_SPIFFS
        type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        mSerial.println("Start updating " + type);

        for (auto& cb : ota_callback_functions) {
            cb();
        }

        // Start with 'blocking' ota loop:
        otaBusy = 1;
        led.set(OFF);
    })
    .onEnd([this]() {
        Serial.println("\nEnd");
        otaBusy = 0;
        led.set(ON);
        // Succesful end will reset ESP itself
    })
    .onProgress([this](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));        
        // Have the led fade on 10 times
        led.set((progress / (total / 25))%255);
        
        feedLoopWDT();
    })
    .onError([this](ota_error_t error) {
        mSerial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) mSerial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) mSerial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) mSerial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) mSerial.println("Receive Failed");
        else if (error == OTA_END_ERROR) mSerial.println("End Failed");
        otaBusy = 0;
        led.set(16);
        
        // Reset in order to have a fresh start on required functionality...
        delay(100);
        ESP.restart();
    });

    ArduinoOTA.begin();
}

// -----------------------------------------------
void WiFiComponent::stopArduinoOTA() {
    ArduinoOTA.end();
}

// -----------------------------------------------
void WiFiComponent::loadParams() {
    WiFiManagerParameter** params = wm.getParameters();
    uint16_t paramCount = wm.getParametersCount();
    for(uint16_t i = 0; i < paramCount; i++) {
        if(params[i]->getID()) {
            Parameter* p = static_cast<Parameter*>(params[i]);
            if(p) {
                p->initialize();
            }
        }
    }
}

// -----------------------------------------------
void WiFiComponent::storeParams() {
    WiFiManagerParameter** params = wm.getParameters();
    uint16_t paramCount = wm.getParametersCount();
    for(uint16_t i = 0; i < paramCount; i++) {
        if(params[i]->getID()) {
            Parameter* p = static_cast<Parameter*>(params[i]);
            if(p) {
                p->storeValue();
            }
        }
    }
}

// -----------------------------------------------
void WiFiComponent::addCustomHtmlPage(const char* path, customHtmlHandler_t handler) {
    custom_html_page_handlers.emplace_back(path, handler);
}

// -----------------------------------------------
void WiFiComponent::saveWifiCallback(){

}

// -----------------------------------------------
//gets called when WiFiManager enters configuration mode
void WiFiComponent::configModeCallback (WiFiManager *myWiFiManager) {
    // ToDo: stop Bluetooth processing!?
}

// -----------------------------------------------
void WiFiComponent::saveParamCallback(){
    storeParams();

    // Call registered callbacks to have them update the parameters
    for(auto& cb : paramSaved_callback_functions) {
        cb();
    }
    // wm.stopConfigPortal();
}

// -----------------------------------------------
void WiFiComponent::handleRoute(){
  wm.server->send(200, "text/plain", "hello from user code");
}

// -----------------------------------------------
void WiFiComponent::bindServerCallback(){
  wm.server->on("/custom",std::bind(&WiFiComponent::handleRoute,*this));
  // wm.server->on("/info",handleRoute); // you can override wm!
  
  for(auto& page : custom_html_page_handlers) {
      wm.server->on(page.path, page.handler);
  }
}
