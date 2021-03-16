
#include <Arduino.h>
#include <U8g2lib.h>
#include "Adafruit_GFX.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

  // https://www.bluetooth.com/specifications/gatt/
// https://www.bluetooth.com/specifications/gatt/services/
// https://www.bluetooth.com/specifications/gatt/characteristics/

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUUID.h>
#include <BLEUtils.h>
// PLXS  Pulse Oximeter Service
static const BLEUUID blePLXServiceUUID(0x1822U);
static const BLEUUID blePLXContMeasUUID(0x2A5FU);
static const BLEUUID blePLXSpotCheckMeasUUID(0x2A5EU);

static BLEServer *bleServer;
static BLEService *bleService;
static BLECharacteristic *blePLXContMeasChar;

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8X8_SH1106_128X64_NONAME_HW_I2C oled(/* reset=*/ U8X8_PIN_NONE);
//U8G2_PCD8544_84X48_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 33, /* data=*/ 25, /* cs=*/ 19, /* dc=*/ 12, /* reset=*/ 17);  // Nokia 5110 Display

#include <Wire.h>
#include "algorithm_by_RF.h"
#include "MAX30102.h"
//#include "spo2_algorithm.h"

//MAX30105 particleSensor;

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
uint8_t display_buffer[BUFFER_SIZE];  //display sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;
// Interrupt pin
const byte oxiInt = 4; // pin connected to MAX30102 INT
bool clientConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
void onConnect (BLEServer * pServer) {
clientConnected = true;
};
void onDisconnect (BLEServer * pServer) {
clientConnected = false;
}
};

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  Wire.begin(21,22);
//  pinMode(pulseLED, OUTPUT);
 // pinMode(readLED, OUTPUT);
  oled.begin();
  //oled.clearBuffer();
  oled.setFont(u8g2_font_questgiver_tr); //u8x8_font_chroma48medium8_r //u8x8_font_8x13B_1x2_r //u8g2_font_tenstamps_mn
 // oled.drawString(0,0,"Startujem...");
//  oled.refreshDisplay();  
 // delay(1000);
  //oled.setDrawColor(1); 
 ////oled.setFont(u8g2_font_6x10_tf);
////  oled.setFontRefHeightExtendedText();
 //// oled.setDrawColor(1);
  ////oled.setFontPosTop();
  ////oled.setFontDirection(0);

  oled.clearBuffer();
   oled.setCursor(0,10);
  oled.print( "Sivak Oxi/Pulz. Start.");
  oled.sendBuffer();
  //oled.sendBuffer();
  
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);

  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();  //initialize the MAX30102
 
 
  // Initialize sensor
  /*
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
     oled.clear();
  oled.drawString( 0, 10, "Sensor Error!!!");
  oled.sendBuffer();
    while (1);
  }*/

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();

  //Setup to sense up to 18 inches, max LED brightness
  /* from original Maxim file
byte ledBrightness = 15; //Options: 0=Off to 255=50mA
byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();
  */

 setup_ble_gatts();

}

void setup_ble_gatts(void) {
  BLEDevice::init("Sivi Oxi");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  bleService = bleServer->createService(blePLXServiceUUID);

  blePLXContMeasChar = bleService->createCharacteristic(
      blePLXContMeasUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  blePLXContMeasChar->addDescriptor(new BLE2902());
  bleService->start();

  BLEAdvertising *bleAdv = BLEDevice::getAdvertising();
  bleAdv->addServiceUUID(blePLXServiceUUID);
  bleAdv->setScanResponse(true);
  
  // functions that help with iPhone connections issue
  bleAdv->setMinPreferred(0x06);
  bleAdv->setMinPreferred(0x12);
  
  BLEDevice::startAdvertising();

 bleServer-> getAdvertising()->start ();// Starts the discovery of ESP32
}

void update(float hrf, float spo2f) {
  uint16_t tmp;
  uint8_t plxValue[5];

  plxValue[0] = 0x00;

  tmp = (uint16_t)round(spo2f);
  plxValue[1] = tmp & 0xff;
  plxValue[2] = (tmp >> 8) & 0xff;

  tmp = (uint16_t)round(hrf);
  plxValue[3] = tmp & 0xff;
  plxValue[4] = (tmp >> 8) & 0xff;

  blePLXContMeasChar->setValue(plxValue, sizeof(plxValue));
  blePLXContMeasChar->notify();
}

uint32_t tempir=0;
uint32_t tempred=0;

void loop()
{
  while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
   maxim_max30102_read_fifo( &(tempir),&(tempred));  //read from MAX30102 FIFO //swapped values for cheap modules
  if (tempred>20000)  
   {

   float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
uint32_t minir=0;
uint32_t maxir=0;
uint32_t rangeir=0;


  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo( (aun_ir_buffer+i),(aun_red_buffer+i));  //read from MAX30102 FIFO //swapped values for cheap modules
   // Serial.println(aun_red_buffer[i], DEC);
   
    
    if (minir==0) minir=aun_red_buffer[i];
    else if (aun_red_buffer[i]<minir) minir=aun_red_buffer[i];
     if (maxir==0) maxir=aun_red_buffer[i];
     else if (aun_red_buffer[i]>maxir) maxir=aun_red_buffer[i];
    rangeir=(maxir-minir)/29;
    if (rangeir==0) rangeir=1;
 
   
   

/*      Serial.print(aun_red_buffer[i], DEC);
   Serial.print(',');
   Serial.print(minir, DEC);
    Serial.print(',');
   Serial.print(maxir, DEC);
    Serial.print(',');
   Serial.println(rangeir, DEC);*/
   // Serial.print(display_buffer[i], DEC);
   // Serial.print(i, DEC);
  //  Serial.print(F("\t"));
/*   Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F("\t"));
    Serial.print(aun_ir_buffer[i], DEC);    
    Serial.println("");*/

  }

  for (i=0;i<BUFFER_SIZE;i++)
  {
     if (aun_red_buffer[i]>20000)
    {
    long temp=aun_red_buffer[i]-minir;
    temp=temp/rangeir;
    
    if (temp>29) temp=29;
    if (temp<0) temp=0;
    display_buffer[i]=temp;
    }
    else
    display_buffer[i]=0;
    if (i>2) display_buffer[i]=(display_buffer[i-1]+display_buffer[i-2]+display_buffer[i])/3;
  }
   
  oled.clearBuffer();
    for(i=0;i<BUFFER_SIZE;i++)
  {
  oled.drawLine(i+1,64,i+1,63-display_buffer[i]);
  }
    
    
//127000 cca red_buffer
//<1000 when no finger
//Serial.println(BUFFER_SIZE);

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);      

 // Serial.print(n_spo2);
 // Serial.print("\t");
 // Serial.println(n_heart_rate, DEC);
if ((ch_spo2_valid)&&(ch_hr_valid))
{
   // oled.clearBuffer();
   oled.setFont(u8g2_font_questgiver_tr); //u8x8_font_chroma48medium8_r //u8x8_font_8x13B_1x2_r //u8g2_font_tenstamps_mn
    oled.setCursor(0,10);
  oled.print( "Sivak Oxi/Pulz. OK.");
  oled.setFont(u8g2_font_ncenB12_te); //u8x8_font_chroma48medium8_r //u8x8_font_8x13B_1x2_r //u8g2_font_tenstamps_mn
  oled.setCursor(0,28);
  oled.print(n_heart_rate,DEC);
  oled.setFont(u8g2_font_questgiver_tr); 
  oled.print("BPM");
  oled.setCursor(60,28);
  oled.setFont(u8g2_font_ncenB12_te);
  oled.print(n_spo2,1);
  oled.setFont(u8g2_font_questgiver_tr); 
  oled.print("%");
  if (clientConnected) 
  {
    oled.setCursor(70,37);
  oled.print("BLE");
  }
   oled.sendBuffer();

if (clientConnected) {
   update(n_heart_rate, n_spo2);
}
 // oled.sendBuffer();
}
else
{
   //oled.clearBuffer();
   oled.setCursor(0,10);
    oled.setFont(u8g2_font_questgiver_tr);
  oled.print( "Sivak Oxi/Pulz. NG!");
  /*oled.setCursor(0,20);
  oled.print("Meriam ");
  oled.print("BPM");
  oled.setCursor(0,30);
  oled.print("Meriam ");
  oled.print("SPO2");*/

    if (clientConnected) 
  {
    oled.setCursor(70,37);
  oled.print("BLE");
  }
  
   oled.sendBuffer(); 
 //// oled.sendBuffer();
}
   }
   else
   {
     oled.clearBuffer();
      oled.setFont(u8g2_font_questgiver_tr);
     oled.setCursor(0,10);
  oled.print( "Sivak Oxi/Pulz.----");
  oled.setCursor(0,28);
  oled.setFont(u8g2_font_ncenB12_te); 
  oled.print("Prilozte Prst!");
    if (clientConnected) 
  {
    oled.setCursor(70,37);
  oled.print("BLE");
  }
   oled.sendBuffer(); 
   }

}
