#include "Particle.h" 
#include "IoTClassroom_CNM.h"
#include "Button.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Encoder.h"
#include "neopixel.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

SYSTEM_MODE(SEMI_AUTOMATIC);

// NeoPixel setup with SPI1
const int PIXEL_COUNT = 16;
Adafruit_NeoPixel pixel(PIXEL_COUNT, SPI1, WS2812B);  // Using SPI1 

// Encoder setup
Encoder myEnc(D8, D9);
int currentPosition;
int lastPosition;
int pixPos;

// OLED display setup
const int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_RESET);

const int HUE_BULBS[6] = {1, 2, 3, 4, 5, 6};  // Identifiers for 6 Hue bulbs
const int WEMO1 = 0;          // Wemo outlet 1 identifier
const int WEMO2 = 1;          // Wemo outlet 2 identifier
const int BUTTON_PIN = D3;    // Pin for button control
const int MOTOR_PIN = D7;     // Pin for motor control

Button button(BUTTON_PIN);    // Initialize Button object on BUTTON_PIN
bool motorAndWemoOn = false;  // Track on/off state for motor and Wemo
bool isAutomaticMode = true;  // Start in automatic mode

int hueColor;                 // Color variable for Hue lights
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 5000;  // Update interval for temperature and OLED

void PixelFill(int startPixel, int endPixel, uint32_t color);

void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);

    // Initialize WiFi
    WiFi.on();
    WiFi.clearCredentials();
    WiFi.setCredentials("IoTNetwork");
    WiFi.connect();
    while (WiFi.connecting()) {
        Serial.printf(".");
    }
    Serial.printf("\n\n");

    // Initialize BME280 sensor
    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
       // while (1);
    }

    // Initialize OLED display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();

    // Initialize motor
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);  // Start with motor off

    // Initialize NeoPixel
    pixel.begin();
    pixel.setBrightness(255);
    pixel.show();

    // Set initial Wemo and Hue states
    wemoWrite(WEMO1, LOW);
    wemoWrite(WEMO2, LOW);
    
    Serial.println("SKODEN");
}

void loop() {
    // Check for button press to toggle motor and Wemo on/off
    if (button.isClicked()) {
        motorAndWemoOn = !motorAndWemoOn;  // Toggle the on/off state

        // Set motor and Wemo based on motorAndWemoOn state
        digitalWrite(MOTOR_PIN, motorAndWemoOn ? HIGH : LOW);
        wemoWrite(WEMO1, motorAndWemoOn ? HIGH : LOW);
        wemoWrite(WEMO2, motorAndWemoOn ? HIGH : LOW);

        Serial.printf("Motor on D7 and Wemo outlets are now %s\n", motorAndWemoOn ? "ON" : "OFF");
    }

    if (isAutomaticMode) {
        // Automatic mode: Adjust Hue color based on temperature
        if (millis() - lastUpdateTime >= updateInterval) {
            lastUpdateTime = millis();

            float temperature = bme.readTemperature();
            hueColor = temperature <= 10 ? 46920 : temperature <= 20 ? 25500 : temperature <= 30 ? 12750 : 0;

            // Set color for all 6 Hue bulbs
            for (int i = 0; i < 6; i++) {
                setHue(HUE_BULBS[i], true, hueColor, 255, 255);
            }

            // Display time and temperature on OLED
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(0, 0);

            String currentTime = Time.format(Time.now(), "%I:%M %p");
            display.printf("Time: %s\n", currentTime.c_str());
            display.printf("Temp: %.2f C", temperature);
            display.display();
        }
    } else {
        // Manual mode: Use encoder to adjust Hue color directly
        currentPosition = myEnc.read();
        hueColor = map(currentPosition, 0, 1000, 0, 65535);

        // Set color for all 6 Hue bulbs in manual mode
        for (int i = 0; i < 6; i++) {
            setHue(HUE_BULBS[i], true, hueColor, 255, 255);
        }
    }

    // NeoPixel Dial based on Encoder Position
    currentPosition = myEnc.read();
    if (currentPosition != lastPosition) {
        Serial.printf("Encoder Position: %i\n", currentPosition);
        lastPosition = currentPosition;
    }
   
    pixPos = 0.479 * currentPosition;

    pixel.clear();
    PixelFill(0, pixPos, pixel.Color(255, 0, 0));  // Fill with red for example
    pixel.show();
}

// Fill NeoPixels from start to end with a specific color
void PixelFill(int startPixel, int endPixel, uint32_t color) {
    for (int litPixel = startPixel; litPixel <= endPixel && litPixel < PIXEL_COUNT; litPixel++) {
        pixel.setPixelColor(litPixel, color);
    }
    pixel.show();
}
