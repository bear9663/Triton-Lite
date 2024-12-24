/**
 *
 * 20240729:圧力センサの値を求めるプログラムがおかしかったため修正
 *
 *
 * @file TritonLite_main.ino * 
 * @brief Triton-Lite用のプログラム
 * @author Komatsu Takuma
 * @date 2024/12/24
 */

//============================================================
// ライブラリインクルード

// 通信関連
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
// SDカード
#include <SD.h>
// GPS
#include <TinyGPS++.h>

// 温度センサ
#include <DallasTemperature.h>
#include <OneWire.h>
#include <OneWireNg.h>
#include <OneWireNg_BitBang.h>
#include <OneWireNg_Config.h>
#include <OneWireNg_CurrentPlatform.h>
// 水圧
#include <MS5837.h>
// RTC
#include <TimeLib.h>
#include <RTC_RX8025NB.h>

//============================================================
// システム設定構造体
struct SystemConfig {
  uint32_t sup_start_time = 30000; // デフォルト30秒
  uint32_t sup_stop_time = 6000;   // デフォルト6秒
  uint32_t exh_start_time = 30000; // デフォルト30秒
  uint32_t exh_stop_time = 10000;  // デフォルト10秒
} config;

//============================================================
// 各種設定
// 電磁弁開閉時間指定
#define SUP_START_TIME (config.sup_start_time)
#define SUP_STOP_TIME (config.sup_stop_time)
#define EXH_START_TIME (config.exh_start_time)
#define EXH_STOP_TIME (config.exh_stop_time)

//SDカードシールド
const int chipSelect = 10;

//GPS
SoftwareSerial mygps(3, 2);  // RX=2ピン, TX=3ピン秋月とスイッチサイエンスで逆
TinyGPSPlus gps;

//温度センサ
#define ONE_WIRE_BUS 4
#define SENSOR_BIT 8

//水圧センサ
MS5837 DepthSensor;

//気圧センサ
const int in_prs_pin = A0;

//RTC
RTC_RX8025NB rtc;

//LED
const int GREEN = 8;
const int RED = 9;

// 電磁弁
const int8_t valve0 = 6;  // 注入バルブ
const int8_t valve1 = 5;  // 排気バルブ
const int8_t valve2 = 7;  // 加圧バルブ

//============================================================
// 変数定義

//GPS
int altitude, gpssatellites;
String lat, lng;

//GPS時刻修正用
int daysInMonth[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//水温
float temperature = 0;

//気圧センサ
float in_prs_rawdata = 0;
float in_prs_voltage = 0;
float in_prs_pressure = 0;

//水圧センサ
float out_pre_pressure = 0;
float out_pre_depth = 0;
float out_pre_tmp = 0;

//RTC
int rtc_year, rtc_month, rtc_day, rtc_hour, rtc_minute, rtc_second;

//millis
unsigned long miliTime, last_ctrl;

// 電磁弁開閉状況,何かコントロール中か
bool V0, V1, V2, isControling;
int8_t Ctrl_state;
/*
*状態
* 0 V0 CLOSE
* 1 V0 OPEN
* 2 V1 CLOSE
* 3 V1 OPEN
*/

int8_t state;
// 状態(上昇=1,下降=2,加圧=3)

//============================================================
// 関数定義

void CtrlValve();
void acquireSensorData();
void getTemperatureData();
void getGPSData();
bool isLeapYear(int year);
void correctTime();
void writeSDcard();
//void writeSDcard_CTRL();

//============================================================
// 各種関数

//------------------------------------------------------------
// データ取得系

// 基本センサデータ取得（OneWire以外）
void acquireSensorData() {
  // 水圧センサデータの取得
  DepthSensor.read();
  out_pre_pressure = DepthSensor.pressure();
  out_pre_depth = DepthSensor.depth();  //mber
  out_pre_tmp = DepthSensor.temperature();

  // 気圧センサデータの取得
  in_prs_rawdata = analogRead(in_prs_pin);
  in_prs_voltage = in_prs_rawdata / 1024 * 5 - 0.25;
  in_prs_pressure = in_prs_voltage / 4.5 * 30;  //PSI　mbarに変換する場合は68.94をかける
}

// OneWire温度センサデータ取得
void getTemperatureData() {
  // OneWireを初期化
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  sensors.begin();
  
  // 温度センサデータの取得
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);
}

// GPSデータ取得
void getGPSData() {
  // GPSシリアルを開始
  mygps.begin(9600);
  
  // GPSデータの取得（一定時間または有効なデータを受信するまで）
  unsigned long startTime = millis();
  bool timeUpdated = false;
  
  while ((millis() - startTime) < 1000) { // 1秒間データを読み取り
    while (mygps.available()) {
      if (gps.encode(mygps.read())) {
        // 位置情報の更新
        if (gps.location.isUpdated()) {
          lat = String(gps.location.lat(), 6);
          lng = String(gps.location.lng(), 6);
          altitude = gps.altitude.meters();
          gpssatellites = gps.satellites.value();
        }
        
        // 時刻情報の更新
        if (gps.time.isValid() && gps.date.isValid()) {
          // RTCの時刻を補正
          correctTime();
          timeUpdated = true;
        }
      }
    }
    
    // 時刻が更新されたら終了
    if (timeUpdated) {
      break;
    }
  }
  
  // デバッグ出力
  Serial.print("GPS Time: ");
  if (gps.time.isValid()) {
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Invalid");
  }
  
  // GPSシリアルを停止
  mygps.end();
}

// 閏年判定
bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

//RTCの時刻をGPSで補正
void correctTime() {
  miliTime = millis();
  
  if (gps.time.isValid() && gps.date.isValid()) {
    //時刻の修正（UTC -> JST）
    int gps_year = gps.date.year();
    int gps_month = gps.date.month();
    int gps_day = gps.date.day();
    int gps_hour = gps.time.hour() + 9;  // UTC+9
    int gps_minute = gps.time.minute();
    int gps_second = gps.time.second();

    // デバッグ出力
    Serial.print("GPS DateTime: ");
    Serial.print(gps_year);
    Serial.print("/");
    Serial.print(gps_month);
    Serial.print("/");
    Serial.print(gps_day);
    Serial.print(" ");
    Serial.print(gps_hour);
    Serial.print(":");
    Serial.print(gps_minute);
    Serial.print(":");
    Serial.println(gps_second);

    //時刻が24を超える場合の処理
    if (gps_hour >= 24) {
      gps_hour -= 24;
      gps_day += 1;
    }

    //月の日数を考慮して日付を修正
    if (gps_day > daysInMonth[gps_month - 1]) {
      if (!(gps_month == 2 && gps_day == 29 && isLeapYear(gps_year))) {
        gps_day = 1;
        gps_month += 1;
      }
    }

    //月が12を超える場合の処理
    if (gps_month > 12) {
      gps_month = 1;
      gps_year += 1;
    }

    //GPSから取得した時刻をRTCに適用
    rtc.setDateTime(gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second);
    
    // RTCから読み取って確認
    tmElements_t tm = rtc.read();
    rtc_year = tmYearToCalendar(tm.Year);
    rtc_month = tm.Month;
    rtc_day = tm.Day;
    rtc_hour = tm.Hour;
    rtc_minute = tm.Minute;
    rtc_second = tm.Second;
    
    // デバッグ出力G
    Serial.print("RTC Set: ");
    Serial.print(rtc_year);
    Serial.print("/");
    Serial.print(rtc_month);
    Serial.print("/");
    Serial.print(rtc_day);
    Serial.print(" ");
    Serial.print(rtc_hour);
    Serial.print(":");
    Serial.print(rtc_minute);
    Serial.print(":");
    Serial.println(rtc_second);
  }
}
// センサデータ書き込み
void writeSDcard() {
  String Sensorlog = "";
  Sensorlog += miliTime;
  Sensorlog += ",";
  Sensorlog += rtc_year;
  Sensorlog += "/";
  Sensorlog += rtc_month;
  Sensorlog += "/";
  Sensorlog += rtc_day;
  Sensorlog += "-";
  Sensorlog += rtc_hour;
  Sensorlog += ":";
  Sensorlog += rtc_minute;
  Sensorlog += ":";
  Sensorlog += rtc_second;
  Sensorlog += ",DATA,";
  Sensorlog += "LAT,";
  Sensorlog += lat;
  Sensorlog += ",LNG,";
  Sensorlog += lng;
  Sensorlog += ",SATNUM,";
  Sensorlog += gpssatellites;
  Sensorlog += ",PIN_RAW,";
  Sensorlog += in_prs_rawdata;
  Sensorlog += ",PIN_PRS,";
  Sensorlog += in_prs_pressure;
  Sensorlog += ",POUT_PRS,";
  Sensorlog += out_pre_pressure;
  Sensorlog += ",POUT_DEPTH,";
  Sensorlog += out_pre_depth;
  Sensorlog += ",POUT_TMP,";
  Sensorlog += out_pre_tmp;
  Sensorlog += ",TMP,";
  Sensorlog += temperature;
  Sensorlog += ",";

  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(Sensorlog);
    dataFile.close();
    Serial.println(Sensorlog);
    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, LOW);
  } else {
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    delay(2000);
  }
}

//------------------------------------------------------------
// 制御系

// 浮沈用電磁弁制御
void CtrlValve() {
  switch (Ctrl_state) {
    case 0:
      if ((miliTime - last_ctrl) > SUP_START_TIME) {
        digitalWrite(valve0, HIGH);
        V0 = 1;
        isControling = 1;
        Ctrl_state = 1;
        state = 1;  // 浮上中
        last_ctrl = millis();
      }
      break;
    case 1:
      if ((miliTime - last_ctrl) > SUP_STOP_TIME) {
        digitalWrite(valve0, LOW);
        V0 = 0;
        isControling = 1;
        Ctrl_state = 2;
        state = 1;  // 浮上中
        last_ctrl = millis();
      }
      break;
    case 2:
      if ((miliTime - last_ctrl) > EXH_START_TIME) {
        digitalWrite(valve1, HIGH);
        V1 = 1;
        isControling = 1;
        Ctrl_state = 3;
        state = 2;  // 沈降中
        last_ctrl = millis();
      }
      break;
    case 3:
      if ((miliTime - last_ctrl) > EXH_STOP_TIME) {
        digitalWrite(valve1, LOW);
        V1 = 0;
        isControling = 1;
        Ctrl_state = 0;
        state = 2;  // 沈降中
        last_ctrl = millis();
      }
      break;
  }
}

// 電磁弁状況書き込み

/* void writeSDcard_CTRL() {
  String CTRLlog = "";
  CTRLlog += miliTime;
  CTRLlog += ",";
  CTRLlog += rtc_year;
  CTRLlog += "/";
  CTRLlog += rtc_month;
  CTRLlog += "/";
  CTRLlog += rtc_day;
  CTRLlog += "-";
  CTRLlog += rtc_hour;
  CTRLlog += ":";
  CTRLlog += rtc_minute;
  CTRLlog += ":";
  CTRLlog += rtc_second;
  CTRLlog += ",CTRL,";
  CTRLlog += "MSG,";

  if (state == 1) {
    CTRLlog += "UP";
  } else if (state == 2) {
    CTRLlog += "DOWN";
  } else if (state == 3) {
    CTRLlog += "PRESSURE";
  } else {
    CTRLlog += "UNDEFIND";
  }

  CTRLlog += ",V0,";
  CTRLlog += V0;
  CTRLlog += ",V1,";
  CTRLlog += V1;
  CTRLlog += ",V2,";
  CTRLlog += V2;

  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(CTRLlog);
    dataFile.close();
    Serial.println(CTRLlog);
    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, LOW);
  } else {
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    delay(2000);
  }
}
*/
//============================================================
// setup

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
    
  Serial.println("GPS Console Complete!");
  
  //SDカードシールド
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("SD ok");

  // 設定の読み込み
  loadConfig();

  Wire.begin();

  //水圧センサ
  while (!DepthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(5000);
  }
  DepthSensor.setModel(MS5837::MS5837_30BA);
  DepthSensor.setFluidDensity(997);
  //RTC
  rtc.setDateTime(2024, 1, 27, 20, 27, 0);

  pinMode(valve0, OUTPUT);
  pinMode(valve1, OUTPUT);
  pinMode(valve2, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);

  last_ctrl = millis();
  Serial.println("Setup Done");
}

//============================================================
// loop

void loop() {
  // シリアルコマンドの確認
  handleSerialCommand();

  // 現在時刻の更新
  miliTime = millis();

  // 基本センサデータの取得（温度センサ以外）
  acquireSensorData();
  
  // 加圧制御
  if (((in_prs_pressure * 68.94) + 1013.25) < out_pre_pressure) {
    digitalWrite(valve2, HIGH);
    V2 = 1;
    isControling = 1;
    delay(100);
  } else {
    digitalWrite(valve2, LOW);
    V2 = 0;
  }

  // OneWireを終了してGPSデータを取得
  getGPSData();
  
  // GPS取得後に温度センサデータを取得
  getTemperatureData();

  // バルブ制御
  CtrlValve();

  // データ記録
  if (isControling == true) {
   // writeSDcard_CTRL();
    isControling = 0;
    state = 0;
  }
  writeSDcard();
}

//============================================================
// 設定関連の関数

// 設定をSDカードから読み込む
void loadConfig() {
  File configFile = SD.open("config.txt");
  if (configFile) {
    String line;
    while (configFile.available()) {
      line = configFile.readStringUntil('\n');
      int separator = line.indexOf(':');
      if (separator != -1) {
        String key = line.substring(0, separator);
        uint32_t value = line.substring(separator + 1).toInt() * 1000; // 秒からミリ秒に変換
        
        if (key == "SUP_START") config.sup_start_time = value;
        else if (key == "SUP_STOP") config.sup_stop_time = value;
        else if (key == "EXH_START") config.exh_start_time = value;
        else if (key == "EXH_STOP") config.exh_stop_time = value;
      }
    }
    configFile.close();
  }
}

// 設定をSDカードに保存
void saveConfig() {
  File configFile = SD.open("config.txt", FILE_WRITE);
  if (configFile) {
    configFile.println("SUP_START:" + String(config.sup_start_time / 1000));
    configFile.println("SUP_STOP:" + String(config.sup_stop_time / 1000));
    configFile.println("EXH_START:" + String(config.exh_start_time / 1000));
    configFile.println("EXH_STOP:" + String(config.exh_stop_time / 1000));
    configFile.close();
  }
}

// シリアルコマンドの処理
void handleSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.println("Received command: " + command);
    
    int separator = command.indexOf(':');
    if (separator != -1) {
      String cmd = command.substring(0, separator);
      uint32_t value = command.substring(separator + 1).toInt() * 1000;
      
      Serial.println("Command: " + cmd);
      Serial.println("Value: " + String(value));
      
      if (cmd == "SUP_START") {
        config.sup_start_time = value;
        Serial.println("SUP_START set to: " + String(value / 1000) + "s");
      }
      else if (cmd == "SUP_STOP") {
        config.sup_stop_time = value;
        Serial.println("SUP_STOP set to: " + String(value / 1000) + "s");
      }
      else if (cmd == "EXH_START") {
        config.exh_start_time = value;
        Serial.println("EXH_START set to: " + String(value / 1000) + "s");
      }
      else if (cmd == "EXH_STOP") {
        config.exh_stop_time = value;
        Serial.println("EXH_STOP set to: " + String(value / 1000) + "s");
      }
      
      Serial.println("Current settings:");
      Serial.println("SUP_START_TIME: " + String(config.sup_start_time / 1000) + "s");
      Serial.println("SUP_STOP_TIME: " + String(config.sup_stop_time / 1000) + "s");
      Serial.println("EXH_START_TIME: " + String(config.exh_start_time / 1000) + "s");
      Serial.println("EXH_STOP_TIME: " + String(config.exh_stop_time / 1000) + "s");
      
      saveConfig();
    }
  }
}
