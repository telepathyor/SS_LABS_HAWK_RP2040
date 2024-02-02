#include <TCA9548A.h>
#include <SPI.h>
#include <SD.h>
#include <WiFiNINA.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <DFRobot_BMM150.h>
#include <bmm150_defs.h>
#include <Wire.h>

SFE_UBLOX_GNSS myGNSS;
TCA9548A hawkMux;
DFRobot_BMM150_I2C bmm150a(&Wire, I2C_ADDRESS_4); //POD A
DFRobot_BMM150_I2C bmm150b(&Wire, I2C_ADDRESS_4); //POD B
DFRobot_BMM150_I2C bmm150c(&Wire, I2C_ADDRESS_4); //FUTURE POD C
DFRobot_BMM150_I2C bmm150d(&Wire, I2C_ADDRESS_4); //FUTURE POD D

#define mySerial Serial1


unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTime = 0; //Used to calc the actual update rate.
unsigned long updateCount = 0; //Used to calc the actual update rate.
const int sampleRate = 50;
unsigned long lastMagSampleTime = 0;
unsigned long lastGPSSampleTime = 0;
const long gpsSampleRate = 100;
long lastLatitude = 0;
long lastLongitude = 0;
const int chipSelect = 4;
File dataFile;

long prevMagDataAx = 0;
long prevMagDataAy = 0;
long prevMagDataAz = 0;
long prevMagDataBx = 0;
long prevMagDataBy = 0;
long prevMagDataBz = 0;
long prevMagDataCx = 0;
long prevMagDataCy = 0;
long prevMagDataCz = 0;
long prevMagDataDx = 0;
long prevMagDataDy = 0;
long prevMagDataDz = 0;

void setup()
{
  delay(5000);
	Serial.begin(460800);
	Serial.println("HAWK");
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDR, HIGH); //RED
  digitalWrite(LEDG, HIGH); //GREEN
  digitalWrite(LEDB, HIGH); //BLUE

  delay(5000);

  digitalWrite(LEDR, LOW); //RED
  digitalWrite(LEDG, LOW); //GREEN
  digitalWrite(LEDB, LOW); //BLUE

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  dataFile = SD.open("TEST.txt", FILE_WRITE);
  if (dataFile){
    dataFile.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz, ddX2, ddY2, ddZ2, sos");
    dataFile.close();
  }

  Wire.begin();
  Wire.setClock(1000000);
  Wire.setTimeout(100);
  Serial.print("Wire initizalized");

	hawkMux.begin(Wire); // I2C address of MUX is 0x77
  hawkMux.closeAll();


  // Original code...
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW); 
  digitalWrite(LEDB, HIGH);

  // Attempt to connect to GNSS module at different baud rates UART/SERIAL
  do {
      // Trying 38400 baud
      Serial.println("GNSS: trying 38400 baud");
      mySerial.begin(38400);
      if (myGNSS.begin(mySerial) == true) break;

      // If not successful, try 9600 baud
      delay(100);
      Serial.println("GNSS: trying 9600 baud");
      mySerial.begin(9600);
      if (myGNSS.begin(mySerial) == true) {
          Serial.println("GNSS: connected at 9600 baud, switching to 38400");
          myGNSS.setSerialRate(38400);
          delay(100);
      } else {
          // If still not successful, reset the GNSS module and try again after a delay
          myGNSS.factoryReset();
          delay(2000); // Wait a bit before trying again to limit the Serial output
      }
  } while (1);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW); 
  digitalWrite(LEDB, LOW);

  // Check for GPS fix type continuously
  while (myGNSS.getFixType() == 0) {
      delay(1000);
      Serial.println("Waiting for GPS fix type...");
  }
  // GPS Fix Acquired, change LED to green
  Serial.println("GPS Fix Acquired");
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH); // Turn on green LED
  digitalWrite(LEDB, LOW);

  delay(1000);
  Serial.println("GPS Configuration Complete");

  hawkMux.openChannel(0);
  bmm150a.begin();
  Serial.println("BMM150a initizalization success");
  bmm150a.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150a.setPresetMode(BMM150_PRESETMODE_REGULAR);
  bmm150a.setRate(BMM150_DATA_RATE_30HZ);
  bmm150a.setMeasurementXYZ();
  hawkMux.closeChannel(0);
  Serial.println("HAWK PodA Configured");

  hawkMux.openChannel(3);
  bmm150b.begin();
  Serial.println("BMM150b initizalization success");
  bmm150b .setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150b.setPresetMode(BMM150_PRESETMODE_REGULAR);
  bmm150b.setRate(BMM150_DATA_RATE_30HZ);
  bmm150b.setMeasurementXYZ();
  hawkMux.closeChannel(3);
  Serial.println("HAWK PodB Configured");

  delay(1000);

  Serial.println("Bomb's Away!!!");
  Serial.println();
  //Serial.println("NON_ALGORITHM");
  //Serial.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz,");
  Serial.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz, ddX2, ddY2, ddZ2, sos");

  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);



}

void loop()
{
  
	unsigned long currentTime = millis();
  if (currentTime - lastMagSampleTime >= sampleRate)
  {
    lastMagSampleTime = currentTime;
    Serial.print(lastLatitude / 10000000.0, 7);
    Serial.print(", ");
    Serial.print(lastLongitude / 10000000.0, 7);
    Serial.print(", ");
    hawkMux.openChannel(0);
    sBmm150MagData_t magDataA = bmm150a.getGeomagneticData();
    hawkMux.closeChannel(0);
    hawkMux.openChannel(3);
    sBmm150MagData_t magDataB = bmm150b.getGeomagneticData();
    hawkMux.closeChannel(3);
    long magDataAx = magDataA.x;
    long magDataAy = magDataA.y;
    long magDataAz = magDataA.z;
    long magDataBx = magDataB.x;
    long magDataBy = magDataB.y;
    long magDataBz = magDataB.z;
    // Calculate the change (delta) and difference between PODs in magnetic field values
    long deltaAx = magDataA.x - prevMagDataAx;
    long deltaAy = magDataA.y - prevMagDataAy;
    long deltaAz = magDataA.z - prevMagDataAz;
    long deltaBx = magDataB.x - prevMagDataBx;
    long deltaBy = magDataB.y - prevMagDataBy;
    long deltaBz = magDataB.z - prevMagDataBz;
    long deltaDiffX = deltaBx - deltaAx;
    long deltaDiffY = deltaBy - deltaAy;
    long deltaDiffZ = deltaBz - deltaAz;
    long ddX2 = deltaDiffX * deltaDiffX;
    long ddY2 = deltaDiffY * deltaDiffY;
    long ddZ2 = deltaDiffZ * deltaDiffZ;
    long sos = ddX2 + ddY2 + ddZ2;
    //Serial.print("PodA: ");
    Serial.print(magDataAx); Serial.print(", ");
    Serial.print(magDataAy); Serial.print(", ");
    Serial.print(magDataAz); Serial.print(", ");
    //Serial.print("PodB: ");
    Serial.print(magDataBx); Serial.print(", ");
    Serial.print(magDataBy); Serial.print(", ");
    Serial.print(magDataBz); Serial.print(", ");
    // Update the previous values for the next iteration
    prevMagDataAx = magDataA.x;
    prevMagDataAy = magDataA.y;
    prevMagDataAz = magDataA.z;
    prevMagDataBx = magDataB.x;
    prevMagDataBy = magDataB.y;
    prevMagDataBz = magDataB.z;

    dataFile = SD.open("TEST.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print(lastLatitude / 10000000.0, 7);
      dataFile.print(", ");
      dataFile.print(lastLongitude / 10000000.0, 7);
      dataFile.print(", ");
      dataFile.print(magDataAx);
      dataFile.print(", ");
      dataFile.print(magDataAy);
      dataFile.print(", ");
      dataFile.print(magDataAz);
      dataFile.print(", ");
      dataFile.print(magDataBx);
      dataFile.print(", ");
      dataFile.print(magDataBy);
      dataFile.print(", ");
      dataFile.print(magDataBz);
      dataFile.print(", ");
      dataFile.print(ddX2);
      dataFile.print(", ");
      dataFile.print(ddY2);
      dataFile.print(", ");
      dataFile.print(ddZ2);
      dataFile.print(", ");
      dataFile.print(sos);
      dataFile.println();
      dataFile.close();
    }
    Serial.println();
  }
  if (millis() % gpsSampleRate == 0)
  {
    long latitude = myGNSS.getLatitude();
    long longitude = myGNSS.getLongitude();
    lastLatitude = latitude;
    lastLongitude = longitude;
  }
}


