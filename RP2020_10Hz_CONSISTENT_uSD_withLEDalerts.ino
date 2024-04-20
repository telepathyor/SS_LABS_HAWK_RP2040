#include <SD.h>
#include <SPI.h>
#include <DFRobot_BMM150.h>
#include <bmm150_defs.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <Adafruit_GPS.h>


#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
TCA9548A I2CMux;  
DFRobot_BMM150_I2C bmm150a(&Wire, I2C_ADDRESS_4); //POD A
DFRobot_BMM150_I2C bmm150b(&Wire, I2C_ADDRESS_4); //POD B
DFRobot_BMM150_I2C bmm150c(&Wire, I2C_ADDRESS_4); //FUTURE POD C
DFRobot_BMM150_I2C bmm150d(&Wire, I2C_ADDRESS_4); //FUTURE POD D

uint32_t timer = millis();
long prevMagDataAx = 0;
long prevMagDataAy = 0;
long prevMagDataAz = 0;
long prevMagDataBx = 0;
long prevMagDataBy = 0;
long prevMagDataBz = 0;
long lastLatitude = 0;
long lastLongitude = 0;
//long prevMagDataCx = 0;
//long prevMagDataCy = 0;
//long prevMagDataCz = 0;
//long prevMagDataDx = 0;
//long prevMagDataDy = 0;
//long prevMagDataDz = 0;
const int gpsNZpin = 3;
const int noSDpin = 2;
const int activeLED = 4;
const int chipSelect = 10;
File dataFile;

void setup()
{
  delay(5000);

  Serial.begin(115200);
  Serial.println("HAWK - 10Hz GPS with (test) MUX");
  //Wire.begin();
  pinMode(gpsNZpin, OUTPUT); // Configure digital pin 6 as an output
  digitalWrite(gpsNZpin, LOW); // Set digital pin 6 HIGH initially
  pinMode(noSDpin, OUTPUT); // Configure digital pin 6 as an output
  digitalWrite(noSDpin, LOW); // Set digital pin 6 HIGH initially
  pinMode(activeLED, OUTPUT); // Configure digital pin 6 as an output
  digitalWrite(activeLED, LOW); // Set digital pin 6 HIGH initially

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  //GPS.begin(9600);
  if (!GPS.begin(9600)){
    Serial.println("GPS Failed");
    while (1);
    }
  Serial.println("GPS all good");

  GPS.sendCommand("$PGKC147,115200*06");
  //delay(250);
  GPS.sendCommand("$PGKC149,0,115200*06");
  //delay(250);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);


  //
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    digitalWrite(noSDpin, HIGH); // Set digital pin 6 HIGH initially

    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  dataFile = SD.open("hawklog.txt", FILE_WRITE);
  if (dataFile){
    dataFile.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz, ddX2, ddY2, ddZ2, sos");
    dataFile.close();
  }
  //

  I2CMux.begin(Wire);
  I2CMux.closeAll();
  I2CMux.openChannel(0);
  while(bmm150a.begin()){
    Serial.println("pod A failed");
    delay(1000);
  } Serial.println("pod A success");

  //Serial.println("BMM150a initizalization success");
  bmm150a.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150a.setPresetMode(BMM150_PRESETMODE_REGULAR);
  bmm150a.setRate(BMM150_DATA_RATE_10HZ);
  bmm150a.setMeasurementXYZ();
  I2CMux.closeChannel(0);
  Serial.println("HAWK PodA Configured");
  I2CMux.openChannel(1);
  while(bmm150b.begin()){
    Serial.println("pod B failed");
    delay(1000);
  } Serial.println("pod A success");

  //bmm150b.begin();
  //Serial.println("BMM150b initizalization success");
  bmm150b .setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150b.setPresetMode(BMM150_PRESETMODE_REGULAR);
  bmm150b.setRate(BMM150_DATA_RATE_10HZ);
  bmm150b.setMeasurementXYZ();
  I2CMux.closeChannel(1);
  Serial.println("HAWK PodB Configured");

  Serial.println("Bomb's Away!!!");
  Serial.println();
  //Serial.println("NON_ALGORITHM");
  //Serial.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz,");
  Serial.println("LAT_DD, LONG_DD, Ax, Ay, Az, Bx, By, Bz, ddX2, ddY2, ddZ2, sos");

}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!

  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // Check if latitude and longitude are non-zero
  if (GPS.latitudeDegrees != 0 && GPS.longitudeDegrees != 0) 
  {
    digitalWrite(gpsNZpin, LOW); // Set digital pin 6 HIGH
    digitalWrite(activeLED, HIGH);
  } else {
    digitalWrite(gpsNZpin, HIGH); // Set digital pin 6 LOW
    digitalWrite(activeLED, LOW);
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 100) {
    timer = millis(); // reset the timer
    //lastLatitude = (GPS.latitudeDegrees, 7);
    //lastLongitude = (GPS.longitudeDegrees, 7);

    //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(GPS.latitudeDegrees, 7);
    Serial.print(", ");
    //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print(GPS.longitudeDegrees, 7);
    //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    //Serial.print("Angle: "); Serial.println(GPS.angle);
    //Serial.print("Altitude: "); Serial.println(GPS.altitude);
    //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    //Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    Serial.print(", ");

    //MUX0
    I2CMux.openChannel(0);
    //Serial.print("Mux0");
    sBmm150MagData_t magDataA = bmm150a.getGeomagneticData();
    I2CMux.closeChannel(0);
    I2CMux.openChannel(1);
    sBmm150MagData_t magDataB = bmm150b.getGeomagneticData();
    I2CMux.closeChannel(1);
    long magDataBx = magDataB.x;
    long magDataBy = magDataB.y;
    long magDataBz = magDataB.z;


    long magDataAx = magDataA.x;
    long magDataAy = magDataA.y;
    long magDataAz = magDataA.z;

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

  dataFile = SD.open("hawklog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print(GPS.latitudeDegrees, 7);
      dataFile.print(", ");
      dataFile.print(GPS.longitudeDegrees, 7);
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




    Serial.print(magDataAx);
    Serial.print(", ");
    Serial.print(magDataAy);
    Serial.print(", ");
    Serial.print(magDataAz);
    Serial.print(", ");

    Serial.print(magDataBx);
    Serial.print(", ");
    Serial.print(magDataBy);
    Serial.print(", ");
    Serial.print(magDataBz);
    Serial.print(", ");
    Serial.print(ddX2);
    Serial.print(", ");
    Serial.print(ddY2);
    Serial.print(", ");
    Serial.print(ddZ2);
    Serial.print(", ");
    Serial.print(sos);

    Serial.println();

    prevMagDataAx = magDataA.x;
    prevMagDataAy = magDataA.y;
    prevMagDataAz = magDataA.z;
    prevMagDataBx = magDataB.x;
    prevMagDataBy = magDataB.y;
    prevMagDataBz = magDataB.z;
  }
  
}