/**
 *
 * 20240729:圧力センサの値を求めるプログラムがおかしかったため修正
 *
 *
 * @file TritonLite_main.ino * 
 * @brief Triton-Lite用のプログラム（GPS時刻同期機能付き、UIでの浮沈時間調節）
 * @author Ryusei Kamiyama , Shintaro Matsumoto,Komatsu Takuma
 * @date 2024/11/23
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
// 電磁弁開閉時間指定（互換性のため残す）
#define SUP_START_TIME (config.sup_start_time)
#define SUP_STOP_TIME (config.sup_stop_time)
#define EXH_START_TIME (config.exh_start_time)
#define EXH_STOP_TIME (config.exh_stop_time)

// 時刻同期関連
const unsigned long TIME_SYNC_INTERVAL = 3600000;  // 同期間隔（1時間）
const unsigned long TIME_PRINT_INTERVAL = 1000;    // 表示間隔（1秒）
bool gpsTimeSet = false;  // GPS時刻が設定されたかどうかのフラグ
unsigned long lastTimeSync = 0;  // 最後に同期した時刻
unsigned long lastTimePrint = 0;  // 最後に時刻を表示した時刻

//SDカードシールド
const int chipSelect = 10;

//GPS
SoftwareSerial mygps(3, 2);  // RX=3ピン, TX=2ピン
TinyGPSPlus gps;

//温度センサ
#define ONE_WIRE_BUS 4
#define SENSOR_BIT 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

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
// 時刻管理関連の関数

// RTCの現在時刻を表示
void printCurrentTime() {
  tmElements_t tm = rtc.read();
  char timeStr[30];
  sprintf(timeStr, "RTC Time: %04d/%02d/%02d %02d:%02d:%02d", 
          tmYearToCalendar(tm.Year), tm.Month, tm.Day, 
          tm.Hour, tm.Minute, tm.Second);
  Serial.println(timeStr);
}

// GPS時刻の状態を確認
void checkGPSTime() {
  if (gps.time.isValid() && gps.date.isValid()) {
    char gpsTimeStr[30];
    // GPS時刻はUTCなので、JST(+9時間)に変換
    int hour = gps.time.hour() + 9;
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();
    
    // 日付の調整
    if (hour >= 24) {
      hour -= 24;
      day++;
    }
    
    sprintf(gpsTimeStr, "GPS Time(JST): %04d/%02d/%02d %02d:%02d:%02d", 
            year, month, day, hour,
            gps.time.minute(), gps.time.second());
    Serial.println(gpsTimeStr);
  } else {
    Serial.println("GPS time not yet valid");
  }
}

// 同期状態の表示
void printSyncStatus() {
  Serial.print("Sync Status: ");
  if (gpsTimeSet) {
    unsigned long timeSinceSync = (millis() - lastTimeSync) / 1000; // 秒に変換
    Serial.print("Last sync ");
    Serial.print(timeSinceSync);
    Serial.println(" seconds ago");
  } else {
    Serial.println("Not yet synchronized with GPS");
  }
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
        uint32_t value = line.substring(separator + 1).toInt() * 1000;
        
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
      else if (cmd == "EXH_START") {
        config.exh_start_time = value;
        Serial.println("EXH_START set to: " + String(value / 1000) + "s");
      }
      
      Serial.println("Current settings:");
      Serial.println("SUP_START_TIME: " + String(config.sup_start_time / 1000) + "s");
      Serial.println("EXH_START_TIME: " + String(config.exh_start_time / 1000) + "s");
    }
  }
}

//============================================================
// setup

void setup() {
  Serial.begin(9600);
  Serial.println("System starting...");

  //SDカードシールド
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("SD ok");

  // 設定の読み込み
  loadConfig();

  //GPS初期化
  mygps.begin(9600);
  Serial.println("GPS initialized");
  
  // RTCに初期時刻を設定
  rtc.setDateTime(2024, 1, 1, 0, 0, 0);  // 2024/01/01 00:00:00
  Serial.println("RTC initialized with default time: 2024/01/01 00:00:00");

  Wire.begin();

  //水圧センサ
  while (!DepthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(5000);
  }
  DepthSensor.setModel(MS5837::MS5837_30BA);  //センサ型番を設定
  DepthSensor.setFluidDensity(997);           //流体密度(kg/m^3)

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
  // 時刻表示の更新
  if (millis() - lastTimePrint > TIME_PRINT_INTERVAL) {
    printCurrentTime();
    checkGPSTime();
    printSyncStatus();
    Serial.println("------------------------");
    lastTimePrint = millis();
  }

  // GPS時刻の初回設定
  if (!gpsTimeSet) {
    if (gps.time.isValid() && gps.date.isValid()) {
      correctTime();
      gpsTimeSet = true;
      lastTimeSync = millis();
      Serial.println("Initial GPS time sync completed");
    }
  }
  // 定期的な同期
  else if (millis() - lastTimeSync > TIME_SYNC_INTERVAL) {
    if (gps.time.isValid() && gps.date.isValid()) {
      correctTime();
      lastTimeSync = millis();
      Serial.println("RTC re-synchronized with GPS time");
    }
  }

  // シリアルコマンドの確認
  handleSerialCommand();

  // GPSデータが利用可能な場合に処理を実行
  while (mygps.available()) {
    gps.encode(mygps.read());
  }

  // センサーデータの取得と処理
  acquireSensorData();
  if (((in_prs_pressure * 68.94) + 1013.25) < out_pre_pressure) {
    digitalWrite(valve2, HIGH);
    V2 = 1;
    isControling = 1;
    delay(100);
  } else {
    digitalWrite(valve2, LOW);
    V2 = 0;
  }

  // バルブ制御
  CtrlValve();

  // SDカードにデータを書き込む
  if (isControling) {
    writeSDcard_CTRL();
    isControling = 0;
    state = 0;
  }
  writeSDcard();
}

//============================================================
// 各種関数

//------------------------------------------------------------
// データ取得系

// センサデータ取得
void acquireSensorData() {
  // GPSデータの取得
  fetchGPSData();

  //RTCデータの取得
  fetchRTCData();

  // 温度センサデータの取得(必要に応じてエラーハンドリング追加)
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

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


// GPSデータ取得
void fetchGPSData() {
  // GPSデータの取得
  lat = String(gps.location.lat(), 6);
  lng = String(gps.location.lng(), 6);
  altitude = gps.altitude.meters();
  gpssatellites = gps.satellites.value();
}

// 閏年かどうかを判定する関数
bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

// RTCデータ
void fetchRTCData() {
  miliTime = millis();
  tmElements_t tm = rtc.read();
  rtc_year = tmYearToCalendar(tm.Year);
  rtc_month = tm.Month;
  rtc_day = tm.Day;
  rtc_hour = tm.Hour;
  rtc_minute = tm.Minute;
  rtc_second = tm.Second;
}

//RTCの時刻をGPSで補正
void correctTime() {
  int daysInMonth[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  while (1) {
    while (mygps.available()) {
      gps.encode(mygps.read());  //GPSから取得したデータをエンコード
    }
    fetchRTCData();

    if (gps.time.isUpdated()) {

      if (gps.location.isUpdated()) {
        //時刻の修正（UTC -> JST）
        int gps_year = gps.date.year();
        int gps_month = gps.date.month();
        int gps_day = gps.date.day();
        int gps_hour = gps.time.hour() + 9;
        int gps_munute = gps.time.minute();
        int gps_second = gps.time.second();

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

        //GPSから取得した時刻をRTCに適用して時間を合わせる
        rtc.setDateTime(gps_year, gps_month, gps_day, gps_hour, gps_munute, gps_second);
        tmElements_t tm = rtc.read();
        Serial.print("RTCに設定した時刻:");
        char s[20];
        sprintf(s, "%d/%d/%d %d:%d:%d", tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
        Serial.println(s);
        Serial.println("RTC set comp");
        break;
      }
    }
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
void writeSDcard_CTRL() {
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
