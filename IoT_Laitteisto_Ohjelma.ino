/******************************************************************************

Analogiaelektroniikka 2 & Projektiopinnot 3 
IoT laitteiston ohjelma Timo Karppanen & Isto Kinnunen 31.5.2019

LM335 Lämpötila anturi
HIH4000 Kosteus anturi
Bluetooth Low Energy
I2C LCD
*******************************************************************************
*/

#include <SPI.h>
#include <BLEPeripheral.h>
#include <String.h>
#include <Wire.h>
#include <I2C_LCD.h>

I2C_LCD LCD;
uint8_t I2C_LCD_ADDRESS = 0x51; // Määritellään laitteiston osoite

/*Alustetaan pinnit Bluetooth Low Energy:lle */

#define BLE_REQ 10
#define BLE_RDY 2
#define BLE_RST 9

void i2cPrint(String avgTemperature, String avgHumid); // Aliohjelma I2C näytön tulostukseen
const int HIH_4000_PIN = A0; // Kosteusanturin alustus porttiin A0
const int LM335_PIN = A1;   // Lämpotila-anturin alustus porttiin A1

float tempvoltage = 0; // Lämpötila-arvon muuttuja ADC arvon muuttamiseen jännitteeksi
float humvoltage = 0;  // Kosteus arvon muuttuja ADC arvon muuttamiseen jännitteeksi

int ADC_TEMP = 0; // Muuttuja lämpötilan ADC arvolle
int HUM_TEMP = 0; // Muuttuja kosteuden ADC arvolle

float rh = 0; // Kosteusanturin arvo
float celsius = 0; // Lämpötila-anturin arvo

long previousMillis = 0; // Säilyttää tietoa milloin anturia on viimeksi luettu.
long interval = 2000;    // Aikaväli joka määrittää että kuinka useasti sensoria luetaan.

/* Määritellään kuinka useasti toistetaan sensorin arvot */

int keskiarvot = 50;

float Averagetempvoltage = 0; // Muuttuja lämpötilan keskiarvoa varten
float Averagehumidvoltage = 0; // Muuttuja kosteuden keskiarvoa varten

/*Bluetooth atribuuttien alustus */

BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);

/* Luodaan bluetooth palvelut, merkinnät, sekä muut tiedot sää-asemalle */

BLEService weatherService = BLEService("BBB0");

BLEFloatCharacteristic temperatureCharacteristic = BLEFloatCharacteristic("BBB1", BLERead | BLENotify);
BLEDescriptor          temperatureDescriptor = BLEDescriptor("2901", "Temperature");

BLEFloatCharacteristic humidityCharacteristic = BLEFloatCharacteristic("BBB2", BLERead | BLENotify);
BLEDescriptor          humidityDescriptor = BLEDescriptor("2901", "Humidity");

void setup()
 {

  Serial.begin(9600);
  Serial.println(F("Bluetooth Low Energy Sääasema"));

  /* Asetetaan Bluetooth laitteistolle nimet */

  blePeripheral.setLocalName("weather");
  blePeripheral.setDeviceName("laitteisto");

  /* Asetetaan palvelut sekä atribuutit  */

  blePeripheral.setAdvertisedServiceUuid(weatherService.uuid());
  blePeripheral.addAttribute(weatherService);
 
  blePeripheral.addAttribute(temperatureCharacteristic);
  blePeripheral.addAttribute(temperatureDescriptor);
  
  blePeripheral.addAttribute(humidityCharacteristic);
  blePeripheral.addAttribute(humidityDescriptor);

  /* Käynnistetään bluetooth */

  blePeripheral.begin();
  Serial.println("Bluetooth on saatu toimimaan.");
  
 }
 
void loop()
{

 /* Alustetaan bluetoothin radiolle atribuutit */
 blePeripheral.poll();

  /* Määritellään kuinka useasti luetaan anturia */
  
    if (millis() - previousMillis > interval)
        {

                  Luesensorit();
                  
 
                  previousMillis = millis();

         }
 
}

void Luesensorit() // Ohjelma sensoreiden lukemiseen
{

  
for(int i = 0; i < keskiarvot; i++) // Lasketaan anturin arvot 50 kertaa yhteen.
{

 ADC_TEMP = analogRead(LM335_PIN);  // Luetaan portista A1 jännitearvo
 ADC_TEMP = analogRead(HIH_4000_PIN); // Luetaan portista A2 jännitearvo

 /* Muutetaan ADC arvot jännitteeksi */

 tempvoltage = ADC_TEMP * (5.0 / 1023.0);
 humvoltage = ADC_TEMP * (5.0 / 1023.0);

 Averagetempvoltage += tempvoltage;
 Averagehumidvoltage += humvoltage;
}
  /* Lasketaan keskiarvot */
  
  Averagetempvoltage /= keskiarvot;
  Averagehumidvoltage /= keskiarvot;

  celsius = ((28 * Averagetempvoltage) - 40); // Muutetaan jännite celsius arvoksi

  rh = (20 * Averagehumidvoltage); // Muutetaan jännite kosteus arvoksi

  /* Tulostetaan Serial monitoriin arvot */
  
  Serial.println("Lämpotila on...");
  Serial.println(celsius);
  Serial.println("Kosteus on...");
  Serial.println(rh);


  /* Tuoodaan arvot I2C näytölle */

  String avgTemperature, avgHumid;
  avgTemperature = (String)celsius;
  avgHumid = (String)rh;
  i2cPrint(avgTemperature,avgHumid);


  /* Tulostetaan lopulliset arvot */
  
   if (!isnan(celsius) && temperatureCharacteristic.value() != celsius)
    {
    temperatureCharacteristic.setValue(celsius);
    Serial.println(celsius);
    }
  if (!isnan(rh) && humidityCharacteristic.value() != rh)
  {
    humidityCharacteristic.setValue(rh);
    Serial.println(rh);
  }

}
void i2cPrint(String avgTemperature, String avgHumid)
{
  LCD.CleanAll(WHITE);    // Puhdistaa näytön edellisistä merkinnöistä
   
  LCD.FontModeConf(Font_8x16_1, FM_ANL_AAA, BLACK_BAC); // Määritellään fontin koko sekä fontin väri

  
  LCD.CharGotoXY(0,16); // Asetetaan tekstille koordinaatit

    /* Tuolostetaan arvot */
    LCD.print("lampotila on: ");
    LCD.println(avgTemperature);
    LCD.print("kosteus on: ");
    LCD.print(avgHumid);

}
