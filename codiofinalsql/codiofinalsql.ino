#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <esp_system.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerif9pt7b.h>

// ================== DEFINES E CONFIGURAÇÕES ==================

// Pinos para sensores e módulos
#define RXD2 16
#define TXD2 17
#define DHTPIN 4
#define DHTTYPE DHT11
#define PIN_SENSOR_A 34
#define PIN_SENSOR_D 15
#define PIN_SENSOR_B 35
#define UMIDADEPIN 13
#define CHUVAPIN 14
#define DHTPINVOLT 27
#define CS_PIN 5
// ================== DEFINES PARA I2C ==================
// RTC utilizará o objeto global Wire com estes pinos:
#define RTC_SDA 21
#define RTC_SCL 22

// OLED utilizará um barramento I2C separado:
#define OLED_SDA 25
#define OLED_SCL 26

// Configurações do display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1


RTC_DS3231 rtc;
// ================== OBJETOS E VARIÁVEIS GLOBAIS ==================
TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

HardwareSerial neogps(1);
DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
File myFile;

// Dados de rede e servidor
const char* ssid = "wifi_Dhonathas";
const char* password = "12345678";
const char* serverUrl = "http://192.168.89.239/API32.php";

// Intervalos para leituras (em milissegundos)
unsigned long previousMillisChuva = 0;
unsigned long previousMillisDHT = 0;
unsigned long previousMillisUmidadeSolo = 0;
unsigned long previousMillisGPS = 0;
unsigned long previousMillisHTTP = 0;
const long intervalChuva = 3000;
const long intervalDHT = 8000;
const long intervalUmidadeSolo = 8000;
const long intervalGPS = 20000;
const long intervalHTTP = 30000; // Enviar dados a cada 30 segundos

// Variáveis para armazenar leituras
String gpsLatitude = "";
String gpsLongitude = "";
String gpsInfo = "";
String chuvaDigital = "";
String chuvaAnalogico = "";
String data = "";
String hora = "";
float umidadeAr = 0.0;
float temperatura = 0.0;
int umidadeSolo = 0;

// Bitmap do ícone de Wi‑Fi (exemplo)
const unsigned char epd_bitmap_Bitmap[] PROGMEM = {
  0xff, 0xfe, 0xff, 0xfe, 0xf0, 0x1e, 0xc0, 0x06, 0x0f, 0xe0, 0xb8, 0x3a, 0xe0, 0x0e, 0xe7, 0xce, 
  0xfe, 0xfe, 0xf8, 0x3e, 0xfb, 0xbe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xfe, 0xff, 0xfe
};

// Outro bitmap para exibição (exemplo)
const unsigned char epd_bitmap_51319[] PROGMEM = {
  0xf0, 0x1e, 0xc0, 0x06, 0x80, 0x02, 0x80, 0x02, 0xc0, 0x06, 0xc0, 0x06, 0xbe, 0x7a, 0x80, 0x02, 
  0xc0, 0x02, 0xc0, 0x06, 0xfc, 0x7e, 0x87, 0xc2, 0x80, 0x02, 0xc0, 0x06, 0xf0, 0x1e
};

// ================== FUNÇÕES AUXILIARES ==================

void invertBitmap(const unsigned char* inputBitmap, unsigned char* outputBitmap, int width, int height) {
  int size = (width * height) / 8;
  for (int i = 0; i < size; i++) {
    outputBitmap[i] = ~pgm_read_byte(&inputBitmap[i]); // Inverte cada byte
  }
}

void WriteCSV(const char *path, const char *message) {
  myFile = SD.open(path, FILE_APPEND);
  if (myFile) {
    myFile.println(message);
    myFile.close();
  } else {
    Serial.println("Erro ao abrir o arquivo");
  }
}

// ================== SETUP ==================

void setup() {
  // Inicializa comunicação serial
  Serial.begin(115200);
  
  // Configuração dos pinos
  pinMode(PIN_SENSOR_A, INPUT);
  pinMode(PIN_SENSOR_D, INPUT);
  pinMode(UMIDADEPIN, OUTPUT);
  pinMode(CHUVAPIN, OUTPUT);
  pinMode(DHTPINVOLT, OUTPUT);
  digitalWrite(UMIDADEPIN, LOW);

  // Inicializa sensores e módulos
  dht.begin();
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Wire.begin(RTC_SDA, RTC_SCL, 100000);
  // Inicializa o RTC
  if (!rtc.begin()) {
    Serial.println("Não foi possível inicializar o RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC perdeu o poder, configurando o tempo...");
     rtc.adjust(DateTime(_DATE, __TIME_));
  }

  // Inicializa o cartão SD
  if (!SD.begin(CS_PIN)) {
    Serial.println("Falha na inicialização do cartão SD");
    return;
  }
  Serial.println("Cartão SD inicializado com sucesso");
  WriteCSV("/data.csv", "Informações Módulo");
  delay(2000);


// ================== INICIALIZAÇÃO DO DISPLAY OLED ==================
I2C_OLED.begin(OLED_SDA, OLED_SCL, 100000);
if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  Serial.println("Falha ao iniciar o OLED");
  while (1);
}
display.clearDisplay();
display.setFont(&FreeSerif9pt7b);
display.setTextSize(1);
display.setTextColor(SSD1306_WHITE);
display.setCursor(20, 40);
display.println("Hortomatico");

// Inverte o bitmap para exibir na parte direita
unsigned char invertedBitmapDB[15 * 15 / 8]; // Tamanho para 15x15 pixels
invertBitmap(epd_bitmap_51319, invertedBitmapDB, 15, 15);
display.drawBitmap(114, 0, invertedBitmapDB, 15, 15, SSD1306_WHITE);
display.display();

  // ================== CONEXÃO COM WIFI COM FEEDBACK NO OLED ==================
  WiFi.begin(ssid, password);
  unsigned char invertedBitmap[15 * 15 / 8];
  invertBitmap(epd_bitmap_Bitmap, invertedBitmap, 15, 15);
  
  int attemptCount = 0;
  // Enquanto não conectar ou até 20 tentativas (para efeito de exibição)
  while (WiFi.status() != WL_CONNECTED && attemptCount < 20) {
    Serial.print(".");
    // Limpa a área do ícone e redesenha a mensagem fixa
    display.fillRect(0, 0, 15, 15, SSD1306_BLACK);
    display.setCursor(20, 40);
    display.println("Hortomatico");
    // Pisca o ícone a cada meio segundo
    if (attemptCount % 2 == 0) {
      display.drawBitmap(0, 0, invertedBitmap, 15, 15, SSD1306_WHITE);
    }
    display.display();
    delay(500);
    attemptCount++;
  }
  
  // Atualiza o display após o loop de conexão
  display.clearDisplay();
  if (WiFi.status() == WL_CONNECTED) {
    display.drawBitmap(0, 0, invertedBitmap, 15, 15, SSD1306_WHITE);
    display.setCursor(20, 40);
    display.println("Hortomatico");
    display.drawBitmap(114, 0, invertedBitmapDB, 15, 15, SSD1306_WHITE);
    display.display();
    Serial.println("WiFi conectado.");
  } else {
    display.fillRect(0, 0, 15, 15, SSD1306_BLACK);
    display.drawBitmap(0, 0, invertedBitmap, 15, 15, SSD1306_WHITE);
    display.setCursor(20, 40);
    display.println("Hortomatico");
    display.drawBitmap(114, 0, invertedBitmapDB, 15, 15, SSD1306_WHITE);
    display.display();
    Serial.println("Falha ao conectar ao WiFi.");
  }
  
  // O restante do setup do código principal segue normalmente...
}

// ================== LOOP ==================

void loop() {
  unsigned long currentMillis = millis();
  DateTime now = rtc.now();
  data = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  hora = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  // Leitura de chuva
  if (currentMillis - previousMillisChuva >= intervalChuva) {
    previousMillisChuva = currentMillis;

    digitalWrite(CHUVAPIN, HIGH);
    delay(10000);
    chuvaDigital = digitalRead(PIN_SENSOR_D) ? "SEM CHUVA" : "ESTA CHOVENDO";
    chuvaAnalogico = String(analogRead(PIN_SENSOR_A));
    digitalWrite(CHUVAPIN, LOW);

    Serial.println("----------CHUVA-----------");
    Serial.println("Digital: " + chuvaDigital);
    Serial.println("Analógico: " + chuvaAnalogico);
    Serial.println("--------------------------");
    hora = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    Serial.println(hora);
    WriteCSV("/data.csv", (data + ", hora: " + hora + " Chuva: " + chuvaDigital + ", valor analógico: " + chuvaAnalogico).c_str());
  }

  // Leitura do sensor DHT (Temperatura e Umidade do Ar)
  if (currentMillis - previousMillisDHT >= intervalDHT) {
    previousMillisDHT = currentMillis;

    digitalWrite(DHTPINVOLT, HIGH);
    delay(10000);
    umidadeAr = dht.readHumidity();
    temperatura = dht.readTemperature();
    delay(2000);
    umidadeAr = dht.readHumidity();
    temperatura = dht.readTemperature();
    digitalWrite(DHTPINVOLT, LOW);

    Serial.println("Umidade do ar: " + String(umidadeAr) + "%");
    Serial.println("Temperatura: " + String(temperatura, 0) + "C");
    Serial.println("--------------------------");
    hora = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    Serial.println(hora);
    
    WriteCSV("/data.csv", (data + ", hora: " + hora + " Umidade do ar: " + String(umidadeAr) + "% , Temperatura: " + String(temperatura, 0) + "C").c_str());
  }

  // Leitura de umidade do solo
  if (currentMillis - previousMillisUmidadeSolo >= intervalUmidadeSolo) {
    previousMillisUmidadeSolo = currentMillis;
    digitalWrite(UMIDADEPIN, HIGH);
    delay(10000);
    umidadeSolo = analogRead(PIN_SENSOR_B);
    digitalWrite(UMIDADEPIN, LOW);

    Serial.println("Umidade do solo: " + String(umidadeSolo));
    Serial.println("--------------------------");
    hora = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    Serial.println(hora);

    WriteCSV("/data.csv", (data + ", hora: " + hora + " Umidade do solo: " + String(umidadeSolo)).c_str());
  }

  // Leitura do GPS
  if (currentMillis - previousMillisGPS >= intervalGPS) {
    previousMillisGPS = currentMillis;

    while (neogps.available() > 0) {
      gps.encode(neogps.read());

      if (gps.location.isValid()) {
        gpsLatitude = String(gps.location.lat(), 6);
        gpsLongitude = String(gps.location.lng(), 6);

        Serial.println("----------INFORMAÇÕES GPS------------");
        Serial.println("Lat: " + gpsLatitude + ", Lng: " + gpsLongitude);
        Serial.println("--------------------------");

        WriteCSV("/data.csv", (data + ", hora: " + hora + " Lat: " + gpsLatitude + ", Lng: " + gpsLongitude).c_str());
        break;
      } else {
        gpsLatitude = "SEM SINAL";
        gpsLongitude = "SEM SINAL";
      }
    }
  }

  // Envio dos dados via HTTP
  int idDispositivo = 234;
  if (currentMillis - previousMillisHTTP >= intervalHTTP) {
    previousMillisHTTP = currentMillis;

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");

      String jsonPayload = "{ \"id_dispositivo\": \"" + String(idDispositivo) + "\", \"leituras\": [";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Umidade do Solo\", \"valor_leitura\":\"" + String(umidadeSolo) + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Umidade do Ar\", \"valor_leitura\":\"" + String(umidadeAr) + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Temperatura\", \"valor_leitura\":\"" + String(temperatura) + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Chuva Digital\", \"valor_leitura\":\"" + chuvaDigital + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Chuva Analógico\", \"valor_leitura\":\"" + chuvaAnalogico + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"GPS Latitude\", \"valor_leitura\":\"" + gpsLatitude + "\"},";
      jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"GPS Longitude\", \"valor_leitura\":\"" + gpsLongitude + "\"}";
      jsonPayload += "]}";

      Serial.println("JSON Enviado:");
      Serial.println(jsonPayload);

      int httpResponseCode = http.POST(jsonPayload);
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Resposta do servidor: " + response);
      } else {
        Serial.print("Erro no envio: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
}