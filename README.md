#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// 屏幕
LiquidCrystal_I2C lcd(0x27, 16, 2);  // 0x27，16列2行

// 引脚
const int PM_PIN = A0;       // PM2.5传感器模拟输入
const int MQ7_PIN = A2;      // 一氧化碳传感器模拟输入
const int HEATER_PIN = 9;    // MQ-7加热控制
const int PM_LED_PIN = 3;    // PM2.5传感器LED控制
SoftwareSerial co2Serial(4, 5);  // CO2传感器软串口(RX=4,TX=5)

// 报警out
const int BUZZER_PIN = 10;   // 蜂鸣器
const int LED_PIN = 11;      // LED
const int RELAY_PIN = 12;    // 通风

// 安全阈值（自定）
const float PM25_GOOD_MAX = 12.0;  // PM2.5良好空气质量上限(μg/m³)
const float PM25_SAFE_MAX = 35.0;  // PM2.5安全浓度上限(μg/m³)
const float CO_GOOD_MAX = 9.0;     // CO良好空气质量上限(ppm)
const float CO_SAFE_MAX = 35.0;    // CO安全浓度上限(ppm)

// 传感器
#define RL_VALUE 10.0       // 负载电阻值(kΩ)
#define RO_CLEAN_AIR 9.8    // 清洁空气中传感器电阻值(kΩ)
#define CO2_PREHEAT_MS 300000  // CO2传感器预热时间(ms)

unsigned long alarmEndTime = 0;  // 报警结束时间戳


void setup() {
  Serial.begin(9600);
  co2Serial.begin(9600);

  // open
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(PM_LED_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, HIGH);  // 启动MQ-7加热器
  digitalWrite(PM_LED_PIN, HIGH);  // 开启PM2.5 LED

  // output
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);  // 关闭蜂鸣器
  digitalWrite(LED_PIN, LOW);      // 关闭LED
  digitalWrite(RELAY_PIN, LOW);    // 关闭继电器

  // LCDopen
  lcd.init();
  lcd.backlight();
  lcd.print("System Starting");
  
  // sencers

  preheatSensors();
  
  lcd.clear();
}


void loop() {
  // 读取
  float pm25 = readPM25();
  float co = readCO();
  int co2 = readCO2();

  // 质量控制
  checkAirQuality(pm25, co);

  // 显示
  displayData(pm25, co, co2);

  // output
  logSensorData(pm25, co, co2);

  delay(2000);  // 2000ms waiting
}


void preheatSensors() {
  // CH4waiting 
  lcd.setCursor(0, 1);
  lcd.print("Preheating CH4...");
  for (int i = 0; i < 30; i++) {
    delay(1000);
    if (i % 5 == 0) lcd.print(".");
  }
  
  // CO2waiting
  lcd.clear();
  lcd.print("CO2 Preheating");
  for (int i = 30; i < 300; i++) {
    delay(1000);
    if (i % 10 == 0) {
      lcd.setCursor(0, 1);
      int remain = 300 - i;
      lcd.print(remain / 60);
      lcd.print("m ");
      lcd.print(remain % 60);
      lcd.print("s   ");
    }
  }
}


void checkAirQuality(float pm25, float co) {
  bool needVentilation = false;
  bool needAlarm = false;

  //chack
  if (pm25 > PM25_GOOD_MAX || co > CO_GOOD_MAX) {
    needVentilation = true;
  }

  // chack fal
  if (pm25 > PM25_SAFE_MAX || co > CO_SAFE_MAX) {
    needAlarm = true;
    alarmEndTime = millis() + 30000;  // 触发报警后持续30秒
  }

  // contral  air
  digitalWrite(RELAY_PIN, needVentilation ? HIGH : LOW);

  // waiting for  close
  bool stillAlarm = (millis() < alarmEndTime) || needAlarm;
  digitalWrite(LED_PIN, stillAlarm ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, stillAlarm ? LOW : HIGH);
}


void displayData(float pm25, float co, int co2) {
  lcd.clear();
  
  
  lcd.setCursor(0, 0);
  lcd.print("PM:");
  lcd.print((int)pm25);
  if (pm25 > PM25_SAFE_MAX) {
    lcd.print("!!");  
  } 
  else if (pm25 > PM25_GOOD_MAX) {
    lcd.print("!");   
  }

 
  lcd.setCursor(0, 1);
  lcd.print("CO:");
  lcd.print((int)co);
  if (co > CO_SAFE_MAX) {
    lcd.print("!!");  
  } 
  else if (co > CO_GOOD_MAX) {
    lcd.print("!");   
  }

  lcd.print(" CO2:");
  if (co2 >= 0) {
    lcd.print(co2);
  } 
  else {
    lcd.print("ERR"); 
  }
}


void logSensorData(float pm25, float co, int co2) {
  Serial.print("PM2.5: ");
  Serial.print(pm25);
  Serial.print(" μg/m³ | ");
  
  Serial.print("CO: ");
  Serial.print(co);
  Serial.print(" ppm | ");
  
  Serial.print("CO2: ");
  if (co2 >= 0) {
    Serial.print(co2);
  } 
  else {
    Serial.print("ERROR");
  }
  Serial.println(" ppm");
}// lcd output


float readPM25() {
  digitalWrite(PM_LED_PIN, LOW);
  delayMicroseconds(280);
  int val = analogRead(PM_PIN);
  delayMicroseconds(40);
  digitalWrite(PM_LED_PIN, HIGH);
  delayMicroseconds(9680);
  
  float voltage = val * (5.0 / 1024.0);
  float density = (0.17 * voltage - 0.1) * 1000;
  return max(0.0, density);  // for sure write digital
}


float readCO() {
  int val = analogRead(MQ7_PIN);
  float rs = RL_VALUE * ((1023.0 / val) - 1);
  return 10.0 * pow(rs / RO_CLEAN_AIR, -2.0);
}


int readCO2() {
  // for looking
  byte cmd[6] = {0x2C, 0x01, 0x00, 0x00, 0x00, 0x2D};
  co2Serial.write(cmd, 6);
  
  // waiting
  delay(100);
  
  if (co2Serial.available() >= 6) {
    byte buffer[6];
    for (int i = 0; i < 6; i++) {
      buffer[i] = co2Serial.read();
    }
    
  
    byte checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4];
    if (checksum == buffer[5]) {
      //return CO2
      return constrain((buffer[1] << 8) | buffer[2], 350, 2000);
    }
  }
  return -1;  // return fal
}
