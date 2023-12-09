#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <WiFi.h>
#include <lvgl.h>
#include <driver/i2s.h>
#include <ATD3.5-S3.h>

#include "gui/ui.h"
// #include <lv_demos.h>

#define SD_CD_PIN (17)
#define SD_CS_PIN (18)

#define LED_Y_PIN (5)
#define BTN_A_PIN (4)

// Sound
#define I2S_DOUT      47
#define I2S_BCLK      48
#define I2S_LRC       45
i2s_port_t i2s_num = I2S_NUM_0; // i2s port number

uint16_t beep_sound_wave[800]; // 16 kHz sample rate

void beep() {
  size_t out_bytes = 0;
  i2s_write(i2s_num, beep_sound_wave, sizeof(beep_sound_wave), &out_bytes, 100);
}

static void drag_event_handler(lv_event_t * e) {
    lv_obj_t * obj = lv_event_get_target(e);

    lv_indev_t * indev = lv_indev_get_act();
    if(indev == NULL) return;

    lv_point_t vect;
    lv_indev_get_vect(indev, &vect);

    int32_t x = lv_obj_get_x(obj) + vect.x;
    int32_t y = lv_obj_get_y(obj) + vect.y;
    lv_obj_set_pos(obj, x, y);
}

void setup() {
  Serial.begin(115200);

  pinMode(SD_CD_PIN, INPUT_PULLUP);
  pinMode(LED_Y_PIN, OUTPUT);
  digitalWrite(LED_Y_PIN, HIGH);
  pinMode(BTN_A_PIN, INPUT);

  // Gen sound beep
  for (uint16_t i=0;i<800;i++) {
    beep_sound_wave[i] = (i % 16) > 8 ? 0x3FFF : 0x0000;
    // Serial.println(beep_sound_wave[i]);
  }

  static const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  static i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level 1, default 0
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = true
  };

  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin(i2s_num, &pin_config);

  Display.begin(0); // Rotation
  Touch.begin();

  Display.useLVGL();
  Touch.useLVGL();
  
  // lv_demo_music();
  // lv_demo_widgets();
  ui_init();

  // Touch page
  lv_obj_add_event_cb(ui_free_drag_object, drag_event_handler, LV_EVENT_PRESSING, NULL);

  // Display

  // Sound
  lv_obj_add_event_cb(ui_play_sound_btn, [](lv_event_t * e) {
    beep();
  }, LV_EVENT_CLICKED, NULL);

  // I2C
  lv_obj_add_event_cb(ui_scan_i2c_btn, [](lv_event_t * e) {
    String log = "";
    Wire.begin();
    log += "Scanning...\n";
 
    int nDevices = 0;
    for(int address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      int error = Wire.endTransmission();
  
      if (error == 0) {
        log += "I2C device found at address 0x";
        if (address<16)
          log += "0";
        log += String(address, HEX);
        log += " !\n";
  
        nDevices++;
      } else if (error==4) {
        log += "Unknown error at address 0x";
        if (address<16)
          log += "0";
        log += String(address, HEX);
        log += "\n";
      }    
    }
    if (nDevices == 0) {
      log += "No I2C devices found\n";
    } else {
      log += "done\n";
    }
    lv_label_set_text(ui_i2c_scan_log, log.c_str());
  }, LV_EVENT_CLICKED, NULL);

  // MicroSD Card

  // OTG

  // LED-SW
  lv_obj_add_event_cb(ui_led_btn, [](lv_event_t * e) {
    digitalWrite(LED_Y_PIN, lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED) ? LOW : HIGH);
  }, LV_EVENT_CLICKED, NULL);

  // IO
  // ----

  // WiFi
  lv_obj_add_event_cb(ui_stat_scan_wifi_btn, [](lv_event_t * e) {
    String log = "";

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    log += "Scan start\n";

    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    log += "Scan done\n";
    if (n == 0) {
      log += "no networks found";
    } else {
      log += String(n);
      log += " networks found\n";
      log += "Nr | SSID | RSSI | CH | Encryption\n";
      for (int i = 0; i < n; ++i) {
        log += String(i + 1);
        log += " | ";
        log += WiFi.SSID(i);
        log += " | ";
        log += String(WiFi.RSSI(i));
        log += " | ";
        log += String(WiFi.channel(i));
        log += " | ";
        switch (WiFi.encryptionType(i)) {
            case WIFI_AUTH_OPEN:
                log += "open";
                break;
            case WIFI_AUTH_WEP:
                log += "WEP";
                break;
            case WIFI_AUTH_WPA_PSK:
                log += "WPA";
                break;
            case WIFI_AUTH_WPA2_PSK:
                log += "WPA2";
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                log += "WPA+WPA2";
                break;
            case WIFI_AUTH_WPA2_ENTERPRISE:
                log += "WPA2-EAP";
                break;
            case WIFI_AUTH_WPA3_PSK:
                log += "WPA3";
                break;
            case WIFI_AUTH_WPA2_WPA3_PSK:
                log += "WPA2+WPA3";
                break;
            case WIFI_AUTH_WAPI_PSK:
                log += "WAPI";
                break;
            default:
                log += "unknown";
        }
        log += "\n";
      }
    }
    log += "\n";

    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();

    lv_label_set_text(ui_wifi_scan_log, log.c_str());
  }, LV_EVENT_CLICKED, NULL);
}

void checkMicroSDCard() {
  if (digitalRead(SD_CD_PIN) == LOW) {
    lv_obj_add_state(ui_microsd_card_detect_sw, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_microsd_card_detect_sw, LV_STATE_CHECKED);
  }
    
  static uint8_t state = 0;
  if (state == 0) {
    if (digitalRead(SD_CD_PIN) == LOW) {
      state = 1;
    } else {
      lv_label_set_text(ui_micro_sd_card_log, "wait insert...");
    }
  } else if (state == 1) {
    String log = "";
    if(SD.begin(SD_CS_PIN)){
      log += "Card Mount OK\n";
      
      uint8_t cardType = SD.cardType();
      if(cardType == CARD_NONE){
        log += "No SD card attached\n";
      } else {
        log += "SD Card Type: ";
        if(cardType == CARD_MMC){
            log += "MMC";
        } else if(cardType == CARD_SD){
            log += "SDSC";
        } else if(cardType == CARD_SDHC){
            log += "SDHC";
        } else {
            log += "UNKNOWN";
        }
        log += "\n";

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        log += "SD Card Size: " + String(cardSize) + " MB\n";
      }
      state = 2;
    } else {
      log += "Card Mount Failed\n";
    }
    lv_label_set_text(ui_micro_sd_card_log, log.c_str());

    if (digitalRead(SD_CD_PIN) == HIGH) {
      state = 0;
    }
  } else if (state == 2) {
    if (digitalRead(SD_CD_PIN) == HIGH) {
      state = 0;
    }
  }
}

void checkBTN_A() {
  if (digitalRead(BTN_A_PIN) == LOW) {
    lv_obj_add_state(ui_switch_y_status, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_switch_y_status, LV_STATE_CHECKED);
  }
}

void loop() {
  Display.loop();
  delay(5);
  checkMicroSDCard();
  checkBTN_A();
}
