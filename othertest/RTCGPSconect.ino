// 設定をSDカードから読み込む
void loadConfig() {
  File configFile = SD.open("config.txt");
  if (configFile) {
    String line;
    while (configFile.available()) {
      char buffer[32];
      size_t len = configFile.readBytesUntil('\n', buffer, sizeof(buffer)-1);
      buffer[len] = '\0';
      line = String(buffer);
      
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
    Serial.println("Config loaded successfully");
  } else {
    Serial.println("Failed to open config.txt");
    // デフォルト値を使用
    config.sup_start_time = 30000;
    config.sup_stop_time = 6000;
    config.exh_start_time = 30000;
    config.exh_stop_time = 10000;
  }
}

// 設定をSDカードに保存
void saveConfig() {
  // 既存のファイルを削除
  if (SD.exists("config.txt")) {
    SD.remove("config.txt");
  }
  
  File configFile = SD.open("config.txt", FILE_WRITE);
  if (configFile) {
    configFile.println("SUP_START:" + String(config.sup_start_time / 1000));
    configFile.println("SUP_STOP:" + String(config.sup_stop_time / 1000));
    configFile.println("EXH_START:" + String(config.exh_start_time / 1000));
    configFile.println("EXH_STOP:" + String(config.exh_stop_time / 1000));
    configFile.close();
    Serial.println("Config saved successfully");
  } else {
    Serial.println("Failed to create config.txt");
  }
}
