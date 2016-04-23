#include <LBT.h>
#include <LBTClient.h>
#include <LBTServer.h>

#include <Timer.h>
#include <HttpClient.h>
#include <LTask.h>
#include <LWiFi.h>
#include <LWiFiClient.h>
#include <LDateTime.h>
#include <LGPS.h>
#include <LBattery.h>
#include "LGATTSUart.h"
#include <LGATTUUID.h>
#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include <String.h>
#define sample_num_mdate  5000


MPU9250 sensorMPU9250;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

int hcount = 0;
float heading_buf[5];

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

bool x = false, s = true;
bool isFall = false;
bool stateAfterDetect = false;
int countInLoop = 0;
int state = 0;
int state_buf = 0;
//int step_count = 0;
int tmp_step = 0;


//#define WIFI_AP "35Y74783"
//#define WIFI_PASSWORD "0972521379"
//#define WIFI_AP "Chiahsunwu"
//#define WIFI_PASSWORD "00000000"
#define WIFI_AP "GG's iPhone"
#define WIFI_PASSWORD "12345678"

#define WIFI_AUTH LWIFI_WPA  // choose from LWIFI_OPEN, LWIFI_WPA, or LWIFI_WEP.
//#define per 50
//#define per1 3
#define DEVICEID "Dw0waaBM" // Input your deviceId
#define DEVICEKEY "WfG3TGkNPoFkfViI" // Input your deviceKey
#define SITE_URL "api.mediatek.com"
//#define DEVICEID "DGLt8oNo"
//#define DEVICEKEY "3TniF090f7oXsR6n" // Input your deviceKey


//String upload_gps;
LGATTSUart uart;
const int buttonPin = 2;
const int buzzerPin = 13;

int buttonState = 0;         // variable for reading the pushbutton status
int onoff = 0;

Timer updateTime;

int dirC = 0;
int steps, stepsAdd = 0;
//bool isFall = 1;
int batteryLv = 0;
float nextLat, nextLong;
float latitudeNow , longitudeNow;
float angle;
int calYN = 0;
bool fallx = 0;

LWiFiClient c2;
HttpClient http(c2);

gpsSentenceInfoStruct info;//gpsSentenceInfoStruct is a structure consist of GPGGA,GPGSA...

///////////////////////////////////////////////////////
//     used when dividing the GPS information        //
///////////////////////////////////////////////////////
const char *nextToken(const char* src, char* buf)
{
  int i = 0;
  while (src[i] != 0 && src[i] != ',')
    i++;
  if (buf)
  {
    strncpy(buf, src, i);
    buf[i] = 0;
  }
  if (src[i])
    i++;
  return src + i;
}
void setup()
{
  LTask.begin();
  LWiFi.begin();
  LGPS.powerOn();
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  //Serial.begin(115200);
  //while(!Serial) delay(1000); /* comment out this line when Serial is not present, ie. run this demo without connect to PC */

  Serial.println("Connecting to AP");
  while (0 == LWiFi.connect(WIFI_AP, LWiFiLoginInfo(WIFI_AUTH, WIFI_PASSWORD)))
  {
    Serial.print("GGGGG");
    delay(1000);
  }
  Serial.println("calling connection");

  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }
  ////////////////////剛開機讀步數///////////////////////
  c2.print("GET /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datachannels/steps/datapoints.csv HTTP/1.1");


  //int dataLength = data.length();

  c2.println("Host: api.mediatek.com");
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  //c2.print("Content-Length: ");
  //c2.println(dataLength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();

  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    /*if (errorcount > 10) {
      c2.stop();
      return;
    }*/
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  int cc = 0;
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      if (cc == 2) {
        steps = steps * 10;
        steps += v - 48;
      }
      if (v == ',')
        cc++;
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }
  Serial.println(steps);
  /////////////////////////////////////////////////////

  Wire.begin();
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  //Serial.begin(19200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  sensorMPU9250.initialize();


  x = sensorMPU9250.getIntEnabled();
  Serial.print("getIntEnabled status :");
  Serial.println(x);


  sensorMPU9250.setIntFreefallEnabled(x);//enable interrupt
  x = sensorMPU9250.getIntFreefallEnabled();


  //delay(1000);
  Serial.println("     ");
  Serial.print("free fall enaable status :");
  Serial.println(x);

  Mxyz_init_calibrated ();

  ////////////////////30秒做一次////////////////////////
  updateTime.every(15000, uploadData);
  /////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////
//                    更新計步                        //
///////////////////////////////////////////////////////
void downloadSteps()
{
  int stepsC = 0;
  LWiFiClient c2;
  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }
  HttpClient http(c2);
  c2.print("GET /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datachannels/steps/datapoints.csv HTTP/1.1");


  //int dataLength = data.length();

  c2.println("Host: api.mediatek.com");
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  //c2.print("Content-Length: ");
  //c2.println(dataLength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();

  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    /*if (errorcount > 10) {
      c2.stop();
      return;
    }*/
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  int cc = 0;
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      if (cc == 2) {
        stepsC = stepsC * 10;
        stepsC += v - 48;
      }
      if (v == ',')
        cc++;
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }
  if (stepsC == 0)
    steps = 0;
  Serial.println("XX");
  Serial.println(stepsC);
  Serial.println(steps);
}
///////////////////////////////////////////////////////
//                 下載下一地點                        //
///////////////////////////////////////////////////////
void downloadNextSpot()
{
  //int stepsC = 0;
  char gpsString[50];
  LWiFiClient c2;
  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }
  HttpClient http(c2);
  c2.print("GET /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datachannels/nextSpot/datapoints.csv HTTP/1.1");


  //int dataLength = data.length();

  c2.println("Host: api.mediatek.com");
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  //c2.print("Content-Length: ");
  //c2.println(dataLength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();

  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    if (errorcount > 10) {
      c2.stop();
      return;
    }
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  int cc = 0, i = 0;
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      if (cc >= 2) {
        char vv = char(v);
        //strcat(gpsString,vv);
        gpsString[i] = vv;
        i++;
        //stepsC = stepsC * 10;
        //stepsC += v - 48;
      }
      if (v == ',')
        cc++;
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }
  //if (stepsC == 0)
  //steps = 0;
  char *latS = strtok(gpsString, ",");
  char *longS = strtok(NULL, ",");
  char *calang = strtok(NULL, ",");
  Serial.println(latS);
  Serial.println(longS);

  Serial.println(calang);
  //char calangg = calang[0];
  if (calang[0] == 48)
    calYN = 0;
  else
    calYN = 1;
//  double aaaaa = 1.445566;
//  Serial.println(aaaaa);
//  Serial.println(aaaaa * 100);
//  Serial.println("GPSSSSSSSSSSSSSSSSS");
//  Serial.println(latS);
//  Serial.println(longS);
//  Serial.println("GPSSSSSSSSSSSSSSSSS");
  nextLat = strtod(latS, NULL);
  nextLong = strtod(longS, NULL);
  Serial.println("GPSSSSSSSSSSSSSSSSS");
  Serial.println(nextLat);
  Serial.println(nextLong);
  Serial.println("GPSSSSSSSSSSSSSSSSS");
}
///////////////////////////////////////////////////////
//           Update the batteryLevel                 //
///////////////////////////////////////////////////////
void uploadBatteryLv()
{
  char upload_batteryLv[] = "batteryLevel,,";
  char batteryC[3];
  sprintf(batteryC, "%d", batteryLv);

  Serial.println("calling connectionn");
  int thislength = strlen(upload_batteryLv) + strlen(batteryC);
  LWiFiClient c2;
  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }

  HttpClient http(c2);
  c2.print("POST /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datapoints.csv HTTP/1.1");
  c2.print("Host: ");
  c2.println(SITE_URL);
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  c2.print("Content-Length: ");
  c2.println(thislength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();
  c2.print(upload_batteryLv);
  c2.println(batteryC);
  //delay(500);


  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    if (errorcount > 10) {
      c2.stop();
      return;
    }
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }
}
///////////////////////////////////////////////////////
//           Update the number of steps              //
///////////////////////////////////////////////////////
void uploadSteps()
{
  char upload_steps[] = "steps,,";
  char stepsC[10];
  sprintf(stepsC, "%d", steps);

  //strcat(upload_steps,stepsC);

  Serial.println("calling connectionn");
  LWiFiClient c2;
  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }

  int thislength = strlen(upload_steps) + strlen(stepsC);
  HttpClient http(c2);
  c2.print("POST /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datapoints.csv HTTP/1.1");
  c2.print("Host: ");
  c2.println(SITE_URL);
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  c2.print("Content-Length: ");
  c2.println(thislength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();
  c2.print(upload_steps);
  c2.println(stepsC);
  //delay(500);

  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    if (errorcount > 10) {
      c2.stop();
      return;
    }
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }
}

///////////////////////////////////////////////////////
//           Update is fall or not                  //
///////////////////////////////////////////////////////
void uploadFallStatus()
{
  char upload_isFall[] = "isFall,,";
  if (isFall == 1)
    strcat(upload_isFall, "1");
  else
    strcat(upload_isFall, "0");

  int thislength = strlen(upload_isFall);

  Serial.println("calling connectionn");
  LWiFiClient c2;
  while (!c2.connect(SITE_URL, 80))
  {
    Serial.println("Re-Connecting to WebSite");
    delay(1000);
  }
  HttpClient http(c2);
  c2.print("POST /mcs/v2/devices/");
  c2.print(DEVICEID);
  c2.println("/datapoints.csv HTTP/1.1");
  c2.print("Host: ");
  c2.println(SITE_URL);
  c2.print("deviceKey: ");
  c2.println(DEVICEKEY);
  c2.print("Content-Length: ");
  c2.println(thislength);
  c2.println("Content-Type: text/csv");
  c2.println("Connection: close");
  c2.println();
  c2.println(upload_isFall);
  //delay(500);

  int errorcount = 0;
  while (!c2.available())
  {
    Serial.print("waiting HTTP response: ");
    Serial.println(errorcount);
    errorcount += 1;
    if (errorcount > 10) {
      c2.stop();
      return;
    }
    delay(100);
  }
  int err = http.skipResponseHeaders();

  int bodyLen = http.contentLength();
  Serial.print("Content length is: ");
  Serial.println(bodyLen);
  Serial.println();
  while (c2)
  {
    int v = c2.read();
    if (v != -1)
    {
      Serial.print(char(v));
    }
    else
    {
      Serial.println("no more content, disconnect");
      c2.stop();

    }
  }

}
void  getNowGPS(const char* str)
{
  char latitude[20];
  char NS[20];
  char NMEAType[20];
  char longitude[20];
  char EW[20];
  char buf[20];
  char height[20];
  const char* p = str;


  p = nextToken(p, NMEAType);
  p = nextToken(p, 0);
  p = nextToken(p, latitude);
  p = nextToken(p, NS);
  p = nextToken(p, longitude);
  p = nextToken(p, EW);
  p = nextToken(p, buf);
  p = nextToken(p, 0);
  p = nextToken(p, 0);
  p = nextToken(p, height);
  double latitudeValue = atof(latitude);
  int latitudeDegree = latitudeValue / 100;
  double latitudeMinute = latitudeValue - 100 * latitudeDegree;

  double longitudeValue = atof(longitude);
  int longitudeDegree = longitudeValue / 100;
  double longitudeMinute = longitudeValue - 100 * longitudeDegree;

  latitudeNow = latitudeDegree + latitudeMinute / 60;
  longitudeNow = longitudeDegree + longitudeMinute / 60;

}

///////////////////////////////////////////////////////
//                Update the GPS                     //
///////////////////////////////////////////////////////
void uploadGPSStatus(const char* str) {
  //getNowGPS(str);
  Serial.println(str);
  char latitude[20];
  char NS[20];
  char NMEAType[20];
  char longitude[20];
  char EW[20];
  char buf[20];
  char height[20];
  const char* p = str;


  p = nextToken(p, NMEAType);
  p = nextToken(p, 0);
  p = nextToken(p, latitude);
  p = nextToken(p, NS);
  p = nextToken(p, longitude);
  p = nextToken(p, EW);
  p = nextToken(p, buf);
  p = nextToken(p, 0);
  p = nextToken(p, 0);
  p = nextToken(p, height);
  double latitudeValue = atof(latitude);
  int latitudeDegree = latitudeValue / 100;
  double latitudeMinute = latitudeValue - 100 * latitudeDegree;

  double longitudeValue = atof(longitude);
  int longitudeDegree = longitudeValue / 100;
  double longitudeMinute = longitudeValue - 100 * longitudeDegree;

  char latitude2[20], longitude2[20];
  sprintf(latitude2, "%f", latitudeNow);
  sprintf(longitude2, "%f", longitudeNow);

  Serial.println(latitude2);
  Serial.println(longitude2);


  if (buf[0] == '1')
  {

    char upload_gps[50];
    strcpy(upload_gps, "GPS,,");
    if (strcmp(NS, "S") == 0)
      strcat(upload_gps, "-");
    Serial.println("3");

    strcat(upload_gps, latitude2);
    strcat(upload_gps, ",");

    if (strcmp(EW, "W") == 0)
      strcat(upload_gps, "-");
    strcat(upload_gps, longitude2);
    strcat(upload_gps, ",");
    strcat(upload_gps, height);

    Serial.print(upload_gps);


    //calling RESTful API to upload datapoint to MCS to report LED status
    Serial.println("calling connectionn");
    LWiFiClient c2;
    while (!c2.connect(SITE_URL, 80))
    {
      Serial.println("Re-Connecting to WebSite");
      delay(1000);
    }

    //String uploadint = "TestInt,,3";
    //upload_gps = "GPS,,1,1,1";

    int thislength = strlen(upload_gps);
    HttpClient http(c2);
    c2.print("POST /mcs/v2/devices/");
    c2.print(DEVICEID);
    c2.println("/datapoints.csv HTTP/1.1");
    c2.print("Host: ");
    c2.println(SITE_URL);
    c2.print("deviceKey: ");
    c2.println(DEVICEKEY);
    c2.print("Content-Length: ");
    c2.println(thislength);
    c2.println("Content-Type: text/csv");
    c2.println("Connection: close");
    c2.println();
    c2.println(upload_gps);
    //delay(500);
    int errorcount = 0;
    while (!c2.available())
    {
      Serial.print("waiting HTTP response: ");
      Serial.println(errorcount);
      errorcount += 1;
      if (errorcount > 10) {
        c2.stop();
        return;
      }
      delay(100);
    }
    int err = http.skipResponseHeaders();

    int bodyLen = http.contentLength();
    Serial.print("Content length is: ");
    Serial.println(bodyLen);
    Serial.println();
    while (c2)
    {
      int v = c2.read();
      if (v != -1)
      {
        Serial.print(char(v));
      }
      else
      {
        Serial.println("no more content, disconnect");
        c2.stop();
      }
    }
  }
  else
  {
    Serial.println(buf[0]);
    Serial.println("GPS is not fixed yet.");
  }
}

void loop()
{
  updateTime.update();
  Serial.println(heading);
  LGPS.getData(&info);
  getNowGPS((char*)info.GPGGA);
  //uploadGPSStatus((char*)info.GPGGA);
  //downloadNextSpot();

  if (calYN == 1)
    calAngle();
  ///////////////////按鈕判斷是否開藍芽//////////////////////
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && onoff == 0) {
    LBTServer.begin((uint8_t*)"Smart Care Shoes");
    LBTServer.end();
    bool scs = LGATTServer.begin(1, &uart);
    onoff = 1;
  }
  else if (buttonState == LOW && onoff == 2 ) {
    LGATTServer.end();
    onoff = 3;
  }
  if (buttonState == HIGH && (onoff == 1 || onoff == 2)) {
    LGATTServer.handleEvents();
    onoff = 2;
  }
  //LGATTServer.handleEvents();

  ////////////////////////////////////////////////////////

  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated();  // compass data has been calibrated here
  getHeading();		      //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();
  if (!isFall) {
    if (Axyz[0] < -1.3) {
      if (Gxyz[0] < -125) {
        Serial.println("fall left");
        isFall = true;
        //tmp_step = steps;
      }
    }
    else if (Axyz[0]  > 1.3) {
      if (Gxyz[0] > 125) {
        Serial.println("fall right");
        isFall = true;
        //tmp_step = steps;
      }
    }
    else if (Axyz[1] < -1.5 ) {
      if (Gxyz[2] > 40) {
        Serial.println("fall front");
        isFall = true;
        //tmp_step = steps;
      }
    }
  }
  /*else {
    if (tmp_step != steps && tmp_step != 0) {
      isFall = false;
      Serial.println("standup");
      tmp_step = 0;
    }
  }*/

  if ( Axyz[2] <= -1.3) {
    stepsAdd++;
  }
  if (hcount == 5) {
    detectDirection();
    hcount = 0;
  }
  //store five values of heading
  heading_buf[hcount++] = heading;
  //Serial.println("XXXXXXXXXXXXXXXXX");
  //Serial.println(isFall);
  if (isFall) {
    uploadFallStatus();
          isFall = false;

  }
  //delay(100);

}
void uploadData()
{
  digitalWrite(buzzerPin, LOW);
  Serial.println("ZZZZZZZZ");
  uploadGPSStatus((char*)info.GPGGA);
 
  downloadSteps();
  steps += stepsAdd;
  stepsAdd = 0;
  uploadSteps();
  downloadNextSpot();
  batteryLv = LBattery.level();
  uploadBatteryLv();
}
void calAngle()
{
  if (nextLat >= latitudeNow && nextLong >= longitudeNow)
    angle = atan((nextLong - longitudeNow) / (nextLat - latitudeNow)) * 180 / 3.1415926; //一
  else if (nextLat <= latitudeNow && nextLong >= longitudeNow)
    angle = 90 + atan(((latitudeNow - nextLat)) / ((nextLong - longitudeNow))) * 180 / 3.1415926; //二
  else if (nextLat >= latitudeNow && nextLong <= longitudeNow)
    angle = 360 - atan(((longitudeNow - nextLong)) / ((nextLat - latitudeNow))) * 180 / 3.1415926; //四
  else if (nextLat <= latitudeNow && nextLong <= longitudeNow)
    angle = 180 + atan(((longitudeNow - nextLong)) / ((latitudeNow - nextLat))) * 180 / 3.1415926;
  //Serial.println("angle");
  //Serial.println(angle);

  if ((int(heading - angle) + 360) % 360 <= 25)
  {
    digitalWrite(buzzerPin, LOW);
    Serial.println("correct");
  }
  else if (((int(angle - heading) + 360) % 360) <= 25)
  {
    digitalWrite(buzzerPin, LOW);
    Serial.println("correct");
  }
  else if (((int(angle - heading) + 360) % 360) <= 180)
  {
    digitalWrite(buzzerPin, HIGH);
    Serial.println("right");
  }
  else
  {
    dirC++;
    if (dirC <= 5)
      digitalWrite(buzzerPin, HIGH);
    else if (dirC > 5 && dirC <= 10)
      digitalWrite(buzzerPin, LOW);
    else
      dirC = 0;

    //Serial.println(dirC);
    Serial.println("left");
  }
}

void print9dofValue()
{
  Serial.print(Axyz[0]);
  Serial.print(" , ");
  Serial.print(Axyz[1]);
  Serial.print(" , ");
  Serial.print(Axyz[2]);
  Serial.print(" , ");
  //Serial.print("Gyro(degress/s) of X,Y,Z:");
  Serial.print(Gxyz[0]);
  Serial.print(" , ");
  Serial.print(Gxyz[1]);
  Serial.print(" , ");
  Serial.print(Gxyz[2]);

  Serial.print(",");
  //Serial.println("Compass Value of X,Y,Z:");
  Serial.print(" , ");
  Serial.print(Mxyz[0]);
  Serial.print(" , ");
  Serial.print(Mxyz[1]);
  Serial.print(" , ");
  Serial.print(Mxyz[2]);
  //Serial.println("The clockwise angle between the magnetic north and X-Axis:");
  Serial.print(" , ");
  Serial.println(heading);
}
bool recordStateAfterDetectingFall()
{
  if (Gxyz[2] > 0)
    return true;
  else
    return false;
}
int getStepCount()
{
  return steps;
}
void detectDirection()
{
  float totalavg = 0.0;
  for (int i = 0; i < 5; i++) {
    totalavg = totalavg + heading_buf[i];
  }
  totalavg = totalavg / 5;
  //Serial.print("avg heading=");
  //Serial.println(totalavg);
  state_buf = state;
  //heading
  if ((totalavg >= 315 && totalavg <= 360) || (totalavg >= 0 && totalavg <= 45) ) //north
    state = 0;
  else if (totalavg >= 45 && totalavg <= 135) //east
    state = 1;
  else if (totalavg >= 135 && totalavg <= 225) //south
    state = 2;
  else if (totalavg >= 225 && totalavg <= 315) //west
    state = 3;

  if (state != state_buf) {
    if (state == 0) Serial.println("North");
    else if (state == 1) Serial.println("East");
    else if (state == 2) Serial.println("South");
    else Serial.println("West");
  }
}
void getFallStatus()
{
  x = sensorMPU9250.getIntFreefallStatus();
  Serial.println(x);
}

void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated ()
{
  mx_centre = -7.0;
  my_centre = 26.0;
  mz_centre = -29.0;
  //	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  //	Serial.print("  ");
  //	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  //	Serial.print("  ");
  //	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  //	while(!Serial.find("ready"));
  //	Serial.println("  ");
  //	Serial.println("ready");
  //	Serial.println("Sample starting......");
  //	Serial.println("waiting ......");
  //
  //	get_calibration_Data ();
  //
  //	Serial.println("     ");
  //	Serial.println("compass calibration parameter ");
  //	Serial.print(mx_centre);
  //	Serial.print("     ");
  //	Serial.print(my_centre);
  //	Serial.print("     ");
  //	Serial.println(mz_centre);
  //	Serial.println("    ");
}
void get_calibration_Data ()
{
  //		for (int i=0; i<sample_num_mdate;i++)
  //		{
  //			get_one_sample_date_mxyz();
  //			/*
  //			Serial.print(mx_sample[2]);
  //			Serial.print(" ");
  //			Serial.print(my_sample[2]);
  //			Serial.print(" ");
  //			Serial.println(mz_sample[2]);
  //			*/
  //                        //find max value
  //			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];
  //			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2];
  //			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];
  //			//find min value
  //			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
  //			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];
  //			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
  //		}
  //
  //			mx_max = mx_sample[1];
  //			my_max = my_sample[1];
  //			mz_max = mz_sample[1];
  //
  //			mx_min = mx_sample[0];
  //			my_min = my_sample[0];
  //			mz_min = mz_sample[0];
  //
  //			mx_centre = (mx_max + mx_min)/2;
  //			my_centre = (my_max + my_min)/2;
  //			mz_centre = (mz_max + mz_min)/2;
}
void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}
void getAccel_Data(void)
{
  sensorMPU9250.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
  sensorMPU9250.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  //Mxyz[0] = (double) mx * 1200 / 4096;
  //Mxyz[1] = (double) my * 1200 / 4096;
  //Mxyz[2] = (double) mz * 1200 / 4096;
  Mxyz[0] = (double) mx * 4800 / 8192;
  Mxyz[1] = (double) my * 4800 / 8192;
  Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}
