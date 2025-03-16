#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <esp_system.h>
#include <RTClib.h> // Biblioteca para RTC

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

HardwareSerial neogps(1);
DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
RTC_DS3231 rtc;
File myFile;

const char* ssid = "A54 de Luciano";
const char* password = "luciano123";
const char* serverUrl = "http://192.168.89.239/API32.php"; 

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


void WriteCSV(const char *path, const char *message) {
  myFile = SD.open(path, FILE_APPEND);
  if (myFile) {
    myFile.println(message);
    myFile.close();
  } else {
    Serial.println("Erro ao abrir o arquivo");
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_SENSOR_A, INPUT);
  pinMode(PIN_SENSOR_D, INPUT);
  pinMode(UMIDADEPIN, OUTPUT);
  pinMode(CHUVAPIN, OUTPUT);
  pinMode(DHTPINVOLT, OUTPUT);
  digitalWrite(UMIDADEPIN, LOW);

  dht.begin();
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  if (!rtc.begin()) {
    Serial.println("Não foi possível inicializar o RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC perdeu o poder, configurando o tempo...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!SD.begin(CS_PIN)) {
    Serial.println("Falha na inicialização do cartão SD");
    return;
  }
  Serial.println("Cartão SD inicializado com sucesso");
  WriteCSV("/data.csv", "Informações Módulo");
  delay(2000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado.");
}

void loop() {
  unsigned long currentMillis = millis();
  DateTime now = rtc.now();
  data = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  hora = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());

    /*if (now.isValid()) {
        Serial.println("Data e hora obtidas com sucesso:");
        Serial.println(String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
        Serial.println(String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
    } else {
        Serial.println("Erro ao obter data e hora do RTC!");
    }
    */
  

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
    Serial.println(String(hora));
    WriteCSV("/data.csv", (data + ", hora: "+ String(hora) + "Chuva: " + chuvaDigital + ", valor analógico: " + chuvaAnalogico).c_str());

  }

  // Leitura de DHT (Temperatura e Umidade)
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
    Serial.println(String(hora));
    
    WriteCSV("/data.csv", (data + ", hora: " + String(hora) + "Umidade do ar: " + String(umidadeAr) + "% , Temperatura: " + String(temperatura, 0) + "C").c_str());
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
    Serial.println(String(hora));

    WriteCSV("/data.csv", (data + ", hora: " + String(hora) + "Umidade do solo: " + String(umidadeSolo)).c_str());
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

        WriteCSV("/data.csv", (data + ", hora: " + String(hora) + "Lat: " + gpsLatitude + ", Lng: " + gpsLongitude).c_str());
        break;
      } else {
        gpsLatitude = "SEM SINAL";
        gpsLongitude = "SEM SINAL";
      }
    }
  }

// Definição do ID do dispositivo
int idDispositivo = 234;

if (currentMillis - previousMillisHTTP >= intervalHTTP) {
    previousMillisHTTP = currentMillis;

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Criando JSON no novo formato com o ID do dispositivo
        String jsonPayload = "{ \"id_dispositivo\": \"" + String(idDispositivo) + "\", \"leituras\": [";

        // Adicionando leituras individuais dos sensores ao JSON
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Umidade do Solo\", \"valor_leitura\":\"" + String(umidadeSolo) + "\"},";
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Umidade do Ar\", \"valor_leitura\":\"" + String(umidadeAr) + "\"},";
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Temperatura\", \"valor_leitura\":\"" + String(temperatura) + "\"},";
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Chuva Digital\", \"valor_leitura\":\"" + chuvaDigital + "\"},";
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"Chuva Analógico\", \"valor_leitura\":\"" + chuvaAnalogico + "\"},";

        // Adicionando coordenadas GPS
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"GPS Latitude\", \"valor_leitura\":\"" + gpsLatitude + "\"},";
        jsonPayload += "{\"data_leitura\":\"" + data + "\", \"hora_leitura\":\"" + hora + "\", \"nome_sensor\":\"GPS Longitude\", \"valor_leitura\":\"" + gpsLongitude + "\"}";

        jsonPayload += "]}"; // Fechando o array e o objeto JSON

        Serial.println("JSON Enviado:");
        Serial.println(jsonPayload);

        // Enviando requisição HTTP
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
