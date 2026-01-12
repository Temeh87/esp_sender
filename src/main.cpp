
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <esp_now.h>

#define ONE_WIRE_BUS 33 // anturi
#define BUTTON_PIN 32 // nappi
#define LED_PIN 25 // LED

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

bool measuring = false;
bool buttonPrevState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastMeasureTime = 0;
const unsigned long measureInterval = 1000;

/** Vastaanottajan MAC **/
uint8_t targetMac[] = {0x78, 0x1C, 0x3C, 0xA7, 0xD8, 0x6C};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}



void setup() {
    
    Serial.begin(115200);
    sensors.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    

    buttonPrevState = digitalRead(BUTTON_PIN);
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW alustus epäonnistui!");
        return;
    }
    esp_now_register_send_cb(OnDataSent);

    Serial.println("ESP-NOW valmis!");

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, targetMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("add_peer failed");
        while (true) delay(1000);
    }

}

void loop() {
    // Lue napin tila
    bool buttonState = digitalRead(BUTTON_PIN);
    unsigned long currentTime = millis();
    
    // Tarkista napin painallus debouncella
    if (buttonState == LOW && buttonPrevState == HIGH) {
        if ((currentTime - lastDebounceTime) > debounceDelay) {

            measuring = !measuring;
            lastDebounceTime = currentTime;
            
            if (measuring) {
                Serial.println("Mittaus aloitettu");
                digitalWrite(LED_PIN, HIGH);
            } else {
                Serial.println("Mittaus pysäytetty");
                digitalWrite(LED_PIN, LOW);
            }
        }
    }

    buttonPrevState = buttonState;

    // Suorita mittaus ja lähetys säännöllisesti, jos mittaus on päällä
    if (measuring && (currentTime - lastMeasureTime) >= measureInterval) {
        lastMeasureTime = currentTime;
        
        sensors.requestTemperatures();
        float temperatureC = sensors.getTempCByIndex(0);
        
        Serial.print("Lämpötila: ");
        Serial.print(temperatureC);
        Serial.println(" °C");
        
        if (esp_now_send(targetMac, (uint8_t *)&temperatureC, sizeof(temperatureC)) != ESP_OK) {
            Serial.println("ESP-NOW lähetys epäonnistui");
        }
    }
}
