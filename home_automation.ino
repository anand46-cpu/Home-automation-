#define BLYNK_TEMPLATE_ID "TMPL3RAF30eKW"
#define BLYNK_TEMPLATE_NAME "HOME"
#define BLYNK_AUTH_TOKEN "uyTJJDuFeHg09TgAJKANciPqUkUSpiGz"

#define BLYNK_PRINT Serial
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <Servo.h>

// Blynk Credentials
char auth[] = "uyTJJDuFeHg09TgAJKANciPqUkUSpiGz";
char ssid[] = "hotspot";
char pass[] = "12345678";

// Sensor Pins
#define MQ135_PIN A0       // MQ135 connected to A0 (Analog pin)
#define DHT_PIN 13          // DHT sensor pin
#define DHT_TYPE DHT11     // DHT11 sensor type
#define LDR_PIN 15          // Digital pin for LDR
#define LED_LDR 14         // LED controlled by LDR
#define LED_BLYNK 12       // LED controlled via Blynk
#define BUZZER_PIN 16      // Buzzer pin (GPIO16/D0)
#define RAIN_SENSOR_PIN 3  // Digital pin for Rain Sensor (GPIO5)
#define SERVO_PIN 2// Servo connected to GPIO4 (D2)

// Variables
DHT dht(DHT_PIN, DHT_TYPE);
BlynkTimer timer;
Servo rainServo;           // Servo object
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize LCD with I2C address 0x27
bool ledBlynkState = false; // Track the state of the Blynk-controlled LED

// Function to read DHT sensor and send data to Blynk
void sendSensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.setCursor(0, 0);
    lcd.print("DHT Read Error");
    lcd.setCursor(0, 1);
    lcd.print("              "); // Clear line
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println(" %");

  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print(" C    ");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print(" %   ");
}

// Function to read MQ135 sensor, activate buzzer, and send data to Blynk
void sendAirQuality() {
  int data = analogRead(MQ135_PIN);
  Serial.print("MQ135 Value: ");
  Serial.println(data);

  Blynk.virtualWrite(V2, data);

  if (data > 505) {
    Serial.println("Poor Air Quality Detected!");
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    lcd.setCursor(0, 1);
    lcd.print("Air Quality Bad!");
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
  }

  // Display MQ135 data on LCD
  lcd.setCursor(0, 0);
  lcd.print("Air Quality: ");
  lcd.print(data);
  lcd.print("     ");
}

// Function to control LED based on LDR reading
void controlLdrLed() {
  int ldrValue = digitalRead(LDR_PIN);
  if (ldrValue == LOW) {
    // Light is absent (LDR is active low)
    digitalWrite(LED_LDR, LOW);
    Serial.println("LDR: Dark, LED ON");
  } else {
    // Light is present
    digitalWrite(LED_LDR, HIGH);
    Serial.println("LDR: Light, LED OFF");
  }
}

// Function to control LED via Blynk switch
BLYNK_WRITE(V3) {
  ledBlynkState = param.asInt(); // Get value from Blynk app (0 or 1)
  digitalWrite(LED_BLYNK, ledBlynkState);
  Serial.print("Blynk LED: ");
  Serial.println(ledBlynkState ? "ON" : "OFF");
}

// Function to read Rain sensor and display on LCD
void checkRain() {
  int rainValue = digitalRead(RAIN_SENSOR_PIN); // Read rain sensor

  if (rainValue == LOW) {
    // Rain detected (Sensor output is LOW when it detects rain)
    Serial.println("Rain Detected!");
    lcd.setCursor(0, 0);
    lcd.print("Rain Detected   ");
    rainServo.write(180); // Turn servo to 90 degrees
  } else {
    // No rain detected
    Serial.println("No Rain Detected");
    lcd.setCursor(0, 0);
    lcd.print("No Rain         ");
    rainServo.write(0);  // Return servo to 0 degrees
  }
}

void setup() {
  Serial.begin(9600);

  // Initialize Blynk
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  // Initialize DHT Sensor
  dht.begin();

  // Initialize Servo
  rainServo.attach(SERVO_PIN);
  rainServo.write(0); // Start servo at 0 degrees

  // Set LED pins as output
  pinMode(LED_LDR, OUTPUT);
  pinMode(LED_BLYNK, OUTPUT);

  // Set LDR pin as input
  pinMode(LDR_PIN, INPUT);

  // Set Buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially

  // Set Rain Sensor pin as input
  pinMode(RAIN_SENSOR_PIN, INPUT);

  // Initialize LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Set timer intervals
  timer.setInterval(3000L, sendSensor);   // Update DHT readings every 3 seconds
  timer.setInterval(5000L, sendAirQuality); // Update MQ135 readings every 5 seconds
  timer.setInterval(1000L, controlLdrLed); // Check LDR state every second
  timer.setInterval(2000L, checkRain);     // Check rain status every 2 seconds
}

void loop() {
  Blynk.run();
  timer.run();
}
