#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
SimpleTimer timer;

// Konfigurasi Wi-Fi
const char *ssid = "Infinix NOTE 30 Pro";
const char *password = "cantique";

// Konfigurasi MQTT
const char *mqtt_server = "broker.hivemq.com";
const char *mqtt_username = "sunpowertech";
const char *mqtt_password = "123";
const char *mqtt_topic = "relay";

// Pin Relay
const int relayPin = D0; //RELAY TERMINAL
const int relay2Pin = D3; //RELAY DHT
const int relay3Pin = D5; //RELAY ATS

// Define analog input
#define ANALOG_IN_PIN A0

// Manual control flag
bool manualControl = false;

// Timestamp untuk memantau waktu terakhir menerima pesan
unsigned long lastMessageTime = 0;

// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;

// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0;

// Float for Reference Voltage
float ref_voltage = 3.3;

// Integer for ADC value
int adc_value = 0;

// Inisialisasi objek untuk PZEM (sensor Utama)
PZEM004Tv30 pzem(D6, D7); // D6 TX, D7 RX

// Variabel penampung untuk nilai sensor Utama
float Voltage, Current, Power;

// Variabel untuk menyimpan jam pemakaian dan total HargaPemakaian
float hour_pemakaian = 0.0;
float totalHargaPemakaian;
float hargaTersimpan;

// Inisialisasi objek untuk DHT21 sensor
#define DHTPIN D4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

// Konstruk object LCD dengan alamat I2C
LiquidCrystal_I2C lcd(0x27, 20, 4); //SCL D1, SDA D2

// Setup RTC dan Serial Print
#include <RTClib.h>
RTC_DS3231 rtc;
int hoursNow;
int minutesNow;
int secondsNow;
int lastMonth = 0;

int eepromAddress = 0; // Alamat EEPROM untuk menyimpan totalHargaPemakaian

void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Update waktu terakhir menerima pesan
  lastMessageTime = millis();

  // Mengendalikan relay berdasarkan pesan yang diterima
  if (message == "ON1") {
    digitalWrite(relayPin, HIGH);
  } else if (message == "OFF1") {
    digitalWrite(relayPin, LOW);
  }

  // Cek apakah ada pesan perintah manual atau tidak
  if (message == "ON2" || message == "OFF2") {
    // Perintah manual dari MQTT payload
    manualControl = true;  // Aktifkan mode manual
    if (message == "ON2" && manualControl == true) {
      digitalWrite(relay3Pin, HIGH);
      Serial.println("Relay ATS ON (Manual Control)");
    } else if (message == "OFF2" && manualControl == true) {
      digitalWrite(relay3Pin, LOW);
      Serial.println("Relay ATS OFF (Manual Control)");
    }
  } else {
    // Perintah otomatis berdasarkan nilai tegangan
    manualControl = false; // Nonaktifkan mode manual
  }  
}

void controlRelay2(float temperature) {
  // Mengendalikan relay2 berdasarkan suhu
  if (temperature >= 35.0) {
    digitalWrite(relay2Pin, HIGH); // Relay 2 ON
  } else {
    digitalWrite(relay2Pin, LOW); // Relay 2 OFF
  }
}

void calculateHarga() {
  unsigned long currentMillis = millis();
  unsigned long previousMillis = 0;
  const long interval = 1000;  // Interval 1 detik (dalam milidetik)

  DateTime now = rtc.now();
  hoursNow = now.hour();
  minutesNow = now.minute();
  secondsNow = now.second();

  // Cek apakah bulan saat ini berbeda dengan bulan terakhir
  if (now.month() != lastMonth) {
    // Reset totalHargaPemakaian menjadi 0 saat awal bulan
    totalHargaPemakaian = 0.0;
    lastMonth = now.month(); // Perbarui bulan terakhir
  }

  // Menghitung totalHargaPemakaian setiap detik
  // Jika ada beban, hour_pemakaian akan terus bertambah
  if (Current > 0) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      hour_pemakaian += 1.0 / 3600.0;  // Penambahan setiap detik
      Serial.print("Hour Pemakaian: ");
      Serial.println(hour_pemakaian);

      // Perhitungan HargaPemakaian per jam
      float hargaPemakaianPerJam = (Power * hour_pemakaian / 1000) * 1444.70;

      // Menambahkan nilai HargaPemakaian per jam ke totalHargaPemakaian
      totalHargaPemakaian += hargaPemakaianPerJam;

      // Menyimpan totalHargaPemakaian ke EEPROM
      EEPROM.put(0, totalHargaPemakaian);
      EEPROM.commit();
    }
  }
  else if (Current == 0) {
    hour_pemakaian = 0.0;
  }
}

void tampilHarga() {
  char buffer[10];
  // Kirim data HargaPemakaian ke MQTT
  if (!isnan(totalHargaPemakaian)) {
    dtostrf(totalHargaPemakaian, 7, 2, buffer); // Konversi float ke string dengan 2 digit desimal
    lcd.setCursor(0, 3);
    lcd.print("Harga : Rp.");
    lcd.print(totalHargaPemakaian);
    Serial.print("Harga : Rp.");
    Serial.print(totalHargaPemakaian);
    client.publish((String(mqtt_topic) + "/price").c_str(), String(totalHargaPemakaian).c_str());
  } else {
    Serial.println("Gagal membaca harga");
  }
  delay(2000);
}

void pzemsensor() {
  // Baca nilai Power (W)
  Power = pzem.power();
  if (!isnan(Power)) {
    lcd.setCursor(0, 0);
    lcd.print("Daya    : ");
    lcd.print(Power);
    lcd.print(" W");
    Serial.print("Power: ");
    Serial.print(Power);
    Serial.println(" Watt");

    // Kirim data ke MQTT
    client.publish((String(mqtt_topic) + "/power").c_str(), String(Power).c_str());
  } else {
    Serial.println("Gagal membaca power");
  }

  // Baca nilai Voltase (V)
  Voltage = pzem.voltage();
  if (!isnan(Voltage)) {
    lcd.setCursor(0, 1);
    lcd.print("Tegangan: ");
    lcd.print(Voltage);
    lcd.print(" V");
    Serial.print("Voltage: ");
    Serial.print(Voltage);
    Serial.println(" Volt");

    // Kirim data ke MQTT
    client.publish((String(mqtt_topic) + "/voltage").c_str(), String(Voltage).c_str());
  } else {
    Serial.println("Gagal membaca voltase");
  }

  // Baca nilai arus (A)
  Current = pzem.current();
  if (!isnan(Current)) {
    lcd.setCursor(0, 2);
    lcd.print("Arus    : ");
    lcd.print(Current);
    lcd.print(" A");
    Serial.print("Arus    : ");
    Serial.print(Current);
    Serial.println(" Ampere");

    // Kirim data ke MQTT
    client.publish((String(mqtt_topic) + "/current").c_str(), String(Current).c_str());
  } else {
    Serial.println("Gagal membaca current");
  }
  calculateHarga();
}

void voltagesensor()
{
  // Membaca Analog Input
  adc_value = analogRead(ANALOG_IN_PIN);

  // Menentukan voltage dengan ADC input
  adc_voltage = (adc_value * ref_voltage) / 1024.0;

  // menghitung voltage dengan divider input
  in_voltage = (adc_voltage / (R2 / (R1 + R2))) - 0.4;

  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);

  // Kirim data ke MQTT
  client.publish((String(mqtt_topic) + "/voltagesensor").c_str(), String(in_voltage).c_str());
  
  // Cek apakah sudah lebih dari 5 detik tanpa pesan masuk
  if (!manualControl && (millis() - lastMessageTime > 5000)) {
    // Perintah otomatis berdasarkan nilai tegangan
    if (in_voltage < 11 && manualControl == false) {
      digitalWrite(relay3Pin, HIGH);
      Serial.println("Switch to PLN (Voltage < 11V)");
    }
    else if (in_voltage == 13.4 && manualControl == false) {
      digitalWrite(relay3Pin, LOW);
      Serial.println("Switch to Battery (Voltage = 13.4V)");
    }
  }
}

void dhtsensor() {
  // Baca nilai suhu dan kelembaban dari sensor DHT21
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // Kirim data suhu dan kelembaban ke MQTT
    client.publish((String(mqtt_topic) + "/temperature").c_str(), String(temperature).c_str());
    client.publish((String(mqtt_topic) + "/humidity").c_str(), String(humidity).c_str());

    // Kontrol relay2 berdasarkan suhu
    controlRelay2(temperature);
  } else {
    Serial.println("Gagal membaca suhu atau kelembaban");
  }
}

void tampilJam() {
  DateTime now = rtc.now();
  hoursNow = now.hour();
  minutesNow = now.minute();
  secondsNow = now.second();
  Serial.print("[");
  Serial.print(hoursNow);
  Serial.print(":");
  Serial.print(minutesNow);
  Serial.print(":");
  Serial.print(secondsNow);
  Serial.println("]");
  Serial.println();
  client.loop();
  //  delay(1000);
}

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);

  // Mulai koneksi Wi-Fi
  Serial.begin(115200);

  // Mulai koneksi wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // Mulai koneksi ke broker MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("NodeMCU-Relay", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }

  // Subscribe ke topik MQTT
  client.subscribe(mqtt_topic);

  // Setup LCD
  Wire.begin(D2, D1); //D2 SDA, D1 SCL
  lcd.begin(20, 4);
  lcd.init();
  lcd.backlight();
  lcd.home();

  // Setup RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC, check your connections!");
    while (1);
  }

  // Baca hargaTersimpan dari EEPROM
  EEPROM.begin(512);
  EEPROM.get(0, hargaTersimpan);
  totalHargaPemakaian = hargaTersimpan;

  // Setup DHT sensor
  dht.begin();
  timer.setInterval(500L, dhtsensor);
  timer.setInterval(500L, voltagesensor);
  timer.setInterval(500L, pzemsensor);
  timer.setInterval(500L, tampilJam);
  timer.setInterval(2000L, tampilHarga);
}

void resetTotalHargaPemakaian() {
  totalHargaPemakaian = 0.0;

  // Simpan nilai reset ke EEPROM
  EEPROM.begin(512);
  EEPROM.put(eepromAddress, totalHargaPemakaian);
  EEPROM.commit();
  EEPROM.end();

  Serial.println("Total Harga Pemakaian direset menjadi Rp.0");
}

void loop() {
  timer.run();

  // Kirim data ke broker MQTT
  tampilHarga();
  tampilJam();
  pzemsensor();
  dhtsensor();
  voltagesensor();
  resetTotalHargaPemakaian();
}
