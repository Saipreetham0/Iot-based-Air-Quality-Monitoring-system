#include <Arduino.h>

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL3KrnFeaQZ"
#define BLYNK_TEMPLATE_NAME "Air Pollution"
#define BLYNK_AUTH_TOKEN "RLHuDxOVS5nEv0djvslgigkUzgbDzB_O"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h> // Library for 16x2 LCD display

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "KSP";
char pass[] = "9550421866";

BlynkTimer timer;

#define MQ2_SENSOR 33

#define MQ135_SENSOR 32

int MQ2_SENSOR_Value = 0;
int MQ135_SENSOR_Value = 0;

float temperature = 0;
float pressure = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD I2C address to 0x27 and 16x2 characters

// This function sends Arduino's uptime every second to Virtual Pin 2.
void uploadData()
{

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, pressure);
  Blynk.virtualWrite(V2, MQ2_SENSOR_Value);
  Blynk.virtualWrite(V3, MQ135_SENSOR_Value);
}

// Initialize the BMP280 sensor
Adafruit_BMP280 bmp;

void connectWiFi()
{
  // Connect to Wi-Fi
  // WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    lcd.setCursor(0, 0);
    lcd.print("WiFi: Connecting...");
  }
  lcd.setCursor(0, 0);
  lcd.print("WiFi: Connected   ");
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  connectWiFi(); // Connect to WiFi network
  unsigned status;
  status = bmp.begin(0x76);

  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));

    while (1)
      delay(10);
  }
  lcd.init();                 // Initialize the LCD
  lcd.backlight();            // Turn on the backlight
  lcd.setCursor(0, 0);        // Set cursor to the first row
  lcd.print("Air Pollution"); // Print message on LCD
  delay(1000);

  // Setup a function to be called every second
  timer.setInterval(1000L, uploadData);
}

void loop()
{
  Blynk.run();
  timer.run();

  // Read temperature and pressure from BMP280
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;

  MQ2_SENSOR_Value = map(analogRead(MQ2_SENSOR), 0, 4095, 0, 100);

  MQ135_SENSOR_Value = analogRead(MQ135_SENSOR);

  Serial.print("MQ2 sensor : ");
  Serial.println(MQ2_SENSOR_Value);
  Serial.print("MQ135 sensor : ");
  Serial.println(MQ135_SENSOR_Value);

  // Update LCD display with sensor readings
  lcd.clear(); // Clear the display

  // Display temperature and pressure
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Press: ");
  lcd.print(pressure);
  lcd.print("hPa");

  delay(5000); // Display temperature and pressure for 5 seconds

  // Clear the display
  lcd.clear();

  // Display MQ2 sensor value
  lcd.setCursor(0, 0);
  lcd.print("MQ2: ");
  lcd.print(MQ2_SENSOR_Value);
  lcd.print("%");

  delay(5000); // Display MQ2 sensor value for 5 seconds

  // Clear the display
  lcd.clear();

  // Display MQ135 sensor value
  lcd.setCursor(0, 0);
  lcd.print("CO2: ");
  lcd.print(MQ135_SENSOR_Value);
  lcd.print(" ppm");

  delay(5000); // Display MQ135 sensor value for 5 seconds

  // Define thresholds for different air quality levels
  int cleanAirThreshold = 100;
  int moderateAirThreshold = 300;
  int poorAirThreshold = 500;

  // Classify air quality based on the sensor value
  if (MQ135_SENSOR_Value < cleanAirThreshold)
  {

    Blynk.logEvent("clean_air");
  }
  else if (MQ135_SENSOR_Value < moderateAirThreshold)
  {
    Blynk.logEvent("moderate_air_quality");
  }
  else if (MQ135_SENSOR_Value < poorAirThreshold)
  {
    Blynk.logEvent("poor_air_quality");
  }
  else
  {
    Blynk.logEvent("very_poor_air_quality");
  }
}
