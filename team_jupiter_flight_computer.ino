/*
  Special thanks to Wifi Web Server LED Blink by Tom Igoe; Roman Silivra from team Mercury AerotechMHS for code for SD card naming for non-wifi testxxx and also .csv file format example
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <RTCZero.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>
#include <Servo.h>
#include <Arduino_LSM6DS3.h>

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "elytraflight";  // your network SSID (name)
char pass[] = "EagleScout";  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key index number (needed only for WEP)

int led = LED_BUILTIN;
int status = WL_IDLE_STATUS;
int udpstatus = 0;
WiFiServer server(80);

unsigned int localPort = 2390;         // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28);  // time.nist.gov NTP server. for whatever reason, this works great with the hotspot.
//IPAddress timeServer(162, 159, 200, 123); //pool.ntp.org NTP server. for whatever reason, this works great with HOME wifi.
const int NTP_PACKET_SIZE = 48;      // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets
WiFiUDP Udp;                         //wifi-related definitions

IPAddress newServer(192, 168, 21, 16);  //the first three numbers MUST MATCH the hotspot. It was 123 beore, now it's 248.


unsigned long epoch = 0;  //seconds since jan 1 2024 12:00:00 AM
int day = 0;
int month = 0;
int yr = 0;
int hr = 0;
int minutes = 0;
int sec = 0;

RTCZero rtc;
//time definitions and stuff




#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmp;
bool BMPresponding = false;
int millisAtLastReading = 0;
//float temperature = 0.0f;
//float pressure = 0.0f;

float altitude = 0.0f;

double bmCum = 0.0f;      //benchmark cumulative sum...get your mind out of the gutter
float bmsTaken = 0.0f;    //benchmarks taken
float bmPressure = 0.0f;  //benchmark pressure
//BMP388 definitions

bool accelResponding = false;
float xaccel;
float yaccel;
float zaccel;
float xgyro;
float ygyro;
float zgyro;



const int SDchipSelect = 10;  //for SD
bool SDresponding = false;
String newfilename = " chosen once the flight computer is armed";
File dataFile;
String extracomments = "";
//SD card definitions



int releaseAngle = 0;
Servo servo;
//Servo definitions

bool usingWifi = true;
bool ARMED = false;
int millisAtLastArm = 0;
bool releaseSoon = false;

int triggerConfidence = 0;

#define triggerAlt 2.0f

#define triggerSkepticism 5  //how many values above the threshold are required before triggering releaseSoon. This prevents a wayward wind gust from triggering the mechanism early. Higher

#define debugMode false

//flight computer related definitions

void setup() {

  if (debugMode) {
    Serial.begin(9600);
    while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
    }
  }


  pinMode(2, OUTPUT);
  pinMode(3, INPUT);

  digitalWrite(2, LOW);
  //Serial.println(digitalRead(3));
  if (!digitalRead(3)) {
    digitalWrite(2, HIGH);
    if (digitalRead(3)) {
      usingWifi = false;
      //Serial.println("yep, shorted");
    }
  }  //this quick check sees if pins 2 and 3 are shorted together, my impromptu way of switching from Wifi to remote AP.


  if (bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    BMPresponding = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    for (int i = 0; i < 10; i++) {
      bmp.performReading();
      delay(10);  //clear out the buffer of junk data that seems to happen right at the start
    }
  }

  if (IMU.begin()) {
    accelResponding = true;
  }

  pinMode(led, OUTPUT);  // set the LED pin mode
  servo.attach(21);      //attach trigger servo

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    //Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }  //check Wifi module and if fail, then blink fast

  if (SD.begin(SDchipSelect)) {
    SDresponding = true;
  }  //check SDresponding and in


  //Serial.println(usingWifi);

  if (usingWifi) {
    // Create open network. Change this line if you want to create an WEP network:

    status = WiFi.begin(ssid, pass);
    while (status != WL_CONNECTED) {
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
      // wait 10 seconds for connection:
      delay(5000);
    }

    Udp.begin(localPort);

    while (!udpstatus) {

      sendNTPpacket(timeServer);  // send an NTP packet to a time server
      delay(1000);

      //Serial.println(udpstatus);
      if (Udp.parsePacket()) {

        //sendNTPpacket(timeServer);  // send an NTP packet to a time server
        //delay(1000);
        //packetLength = Udp.parsePacket();
        udpstatus = 1;
        //Serial.println("test");

        // We've received a packet, read the data from it

        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read the packet into the buffer

        //the timestamp starts at byte 40 of the received packet and is four bytes,

        // or two words, long. First, esxtract the two words:

        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);

        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

        // combine the four bytes (two words) into a long integer

        // this is NTP time (seconds since Jan 1 1900):

        unsigned long secsSince1900 = highWord << 16 | lowWord;

        const unsigned long seventyYears = 2208988800UL;


        epoch = secsSince1900 - seventyYears - 28800;  //28800 because PST is UTC-8. This will have to be changed to 7 hours once we go back to PDT.
        rtc.begin();                                   // initialize RTC
        rtc.setEpoch(epoch);
        day = rtc.getDay();
        month = rtc.getMonth();
        yr = rtc.getYear();
        hr = rtc.getHours();
        minutes = rtc.getMinutes();
        sec = rtc.getSeconds();
        break;

      }  //send a packet to the time service, receive it, and then update the time to rtc. Set rtc epoch so that the arduino can now handle timing henceforth.
      delay(1000);
    }  //get the time before moving on
    WiFi.config(newServer);
  }  //initialize Wifi, initialize RTC and send a time packet to grab the real-world time

  if (!usingWifi) {
    WiFi.config(newServer);
    status = WiFi.beginAP("jupiterAP", pass);
    while (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
      // don't continue
      delay(5000);
      status = WiFi.beginAP("jupiterAP", pass);
    }
  }  //initialize the AP, initialize

  // start the web server on port 80
  server.begin();
}


void loop() {

  if (millis() >= millisAtLastReading + 200) {
    BMPresponding = bmp.performReading();  //perform a BMP reading
    if (!ARMED) {
      bmCum += bmp.pressure;
      bmsTaken += 1.0f;
      bmPressure = bmCum / bmsTaken;  //IN PASCALS, NOT HECTOPASCALS
    }
    if (ARMED) {

      altitude = bmp.readAltitude(bmPressure * 0.01f);
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(xaccel, yaccel, zaccel);
      }
      if (IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(xgyro, ygyro, zgyro);
      }
      dataFile.println(String((millis() - millisAtLastArm) * 0.001f) + "," + String(bmp.temperature) + "," + String(bmp.pressure * 0.01f) + "," + String(altitude) + "," + String(xaccel) + "," + String(yaccel) + "," + String(zaccel) + "," + String(xgyro) + "," + String(ygyro) + "," + String(zgyro) + "," + String(extracomments));
      extracomments = "";

      if (altitude > triggerAlt && !releaseSoon && releaseAngle != 180) {  //if it's above the altitude and it hasn't already triggered releaseSoon once.
        triggerConfidence += 1;
        extracomments += "triggerConfidence is now " + String(triggerConfidence) + ". ";
      }
      if (altitude <= triggerAlt && triggerConfidence > 0) {  //if it went back down below the altitude before it got confident about it.
        triggerConfidence = 0;
        extracomments += "false alarm. trigger confidence is now 0. ";
      }

      if (triggerConfidence >= triggerSkepticism && !releaseSoon && releaseAngle != 180) {
        //multiple readings have now indicated the rocket has gone higher than triggerAlt; go ahead and get ready for a release on the way down
        releaseSoon = true;
        triggerConfidence = 0;
        extracomments += "releaseSoon triggered. ";
      }

      if (releaseSoon && altitude < triggerAlt && releaseAngle != 180) {
        //this SHOULD happen on the way back down. The rocket has hit the triggerAlt on the way back down; release chute now
        releaseAngle = 180;
        releaseSoon = false;
        extracomments += "release triggered. ";
      }
    }
    millisAtLastReading = millis();

    servo.write(releaseAngle);
  }  //bmp reading and SD println
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();
  }

  if (usingWifi) {
    if (status == WL_DISCONNECTED || status == WL_CONNECTION_LOST) {
      if (status != WL_CONNECTED) {
        status = WiFi.begin(ssid, pass);
      }
    }
  }  //attempt to connect to wifi if lost

  WiFiClient client = server.available();  // listen for incoming clients
  //Serial.println(client);

  if (client) {  // if you get a client, send the HTML
    //Serial.println("new client");           // print a message out the serial port
    String currentLine = "";      // make a String to hold incoming data from the client
    while (client.connected()) {  // loop while the client's connected
      delayMicroseconds(10);      // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {   // if there's bytes to read from the client,
        char c = client.read();   // read a byte, then
        //Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {  // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");

            // the content of the HTTP response follows the header:
            printWiFiStatus(client);

            if (usingWifi) {
              client.print("<br> The flight computer is connected via hotspot/wifi to the greater internet.");
            } else {
              client.print("<br> Since pins 2 and 3 are shorted together, the flight computer is NOT connected to the greater internet, and instead has created a remote access point.");
            }

            if (usingWifi) {

              client.print("<br> the time currently is ");

              day = rtc.getDay();
              month = rtc.getMonth();
              yr = rtc.getYear();
              hr = rtc.getHours();
              minutes = rtc.getMinutes();
              sec = rtc.getSeconds();

              client.print(month);
              client.print("/");
              client.print(day);
              client.print("/");
              client.print(yr);
              client.print("\t");  //date

              client.print(hr);
              client.print(":");
              client.print(minutes);
              client.print(":");
              client.print(sec);  //time

              client.print(" Pacific Standard Time. ");  //date and time
            }
            client.print("<br> The name of the SD file for this test run will be ");
            client.print(newfilename);

            client.print(" . <br>");

            client.print("the SD card is ");
            if (!SDresponding) { client.print("NOT "); }
            client.print(" responding. <br>");

            client.print("<br> the altimeter is ");
            if (!BMPresponding) { client.print("NOT "); }
            client.print(" responding.");

            client.print("<br> the accelerometer/gyro is ");
            if (!accelResponding) { client.print("NOT "); }
            client.print(" responding.");

            client.print("<br> The benchmark average pressure is ");
            client.print(bmPressure);
            client.print(" hPa. <br>");

            client.print("<br> The flight computer is ");
            if (!ARMED) { client.print("NOT "); }
            client.print("armed for flight. <br>");  //whether the flight computer is armed or not

            client.print("<br> Click <a href=\"/\">here</a> to refresh the page for new data. <br>");

            if (!ARMED) {
              client.print("<br> Click <a href=\"/0\">here</a> to set the trigger angle to 0, allowing rubber band wrapping. <br>");
              client.print("<br> Click <a href=\"/180\">here</a> to set the trigger to 180, releasing rubber band. <br>");
              client.print("<br> Click <a href=\"/A\">here</a> to ARM the flight computer and prepare for launch <br>");
            }
            if (ARMED) {
              client.print("<br> Click <a href=\"/D\">here</a> to DISARM the flight computer and receive data <br>");  //interaction options
            }


            client.print("<br> disclaimer: do not spam ARM and DISARM more than once per minute, or your data will overwrite to the same filename.");

            // The HTTP response ends with another blank line:
            client.println("</html>");
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /A" or "GET /D":
        if (currentLine.endsWith("GET /A") && !ARMED) {
          digitalWrite(led, HIGH);  // GET /A turns the LED on
          ARMED = true;
          millisAtLastArm = millis();
          releaseSoon = false;
          releaseAngle = 0;

          SDresponding = SD.begin(SDchipSelect);
          newfilename = SDcardFileName();
          dataFile = SD.open(newfilename, FILE_WRITE);
          dataFile.println("Time (s),Temp (C),Pressure (hPa),Altitude (m), accel x (g), accel y (g), accel z (g), omega x (dps), omega y (dps), omega z (dps), comments");  //create a new file. MAKE SURE THIS DOESNT GET CALLED TWICE IN A ROW
        }
        if (currentLine.endsWith("GET /D") && ARMED) {
          digitalWrite(led, LOW);  // GET /D turns the LED off
          ARMED = false;
          releaseSoon = false;
          triggerConfidence = 0;
          releaseAngle = 0;  //back to loaded configuration
          dataFile.close();  //close the currently open file
        }
        if (!ARMED && currentLine.endsWith("GET /0")) {
          releaseAngle = 0;
        }
        if (!ARMED && currentLine.endsWith("GET /180")) {
          releaseAngle = 180;
        }
      }
    }
    // close the connection:
    client.stop();
    //Serial.println("client disconnected");
  }
}

String SDcardFileName() {

  String name = "";

  if (usingWifi) {
    //use the timestamp: MMDDHHMM.csv
    day = rtc.getDay();
    month = rtc.getMonth();
    //yr = rtc.getYear();
    hr = rtc.getHours();
    minutes = rtc.getMinutes();
    if (String(month).length() < 2) { name += "0"; }
    name += String(month);
    if (String(day).length() < 2) { name += "0"; }
    name += String(day);
    if (String(hr).length() < 2) { name += "0"; }
    name += String(hr);
    if (String(minutes).length() < 2) { name += "0"; }
    name += String(minutes);
    name += ".csv";
  } else {
    //name sequentially:jtestxxx
    int SDindex = 0;
    name = String("jtest" + String(SDindex) + ".csv");  //Checking each file on the SD card by pattern TEST1.csv, TEST2.csv, TEST3.csv, etc. until no file is found
    while (SD.exists(name)) {
      SDindex++;
      name = String("jtest" + String(SDindex) + ".csv");  //Checking each file on the SD card by pattern TEST1.csv, TEST2.csv, TEST3.csv, etc. until no file is found
    }
  }
  //sec = rtc.getSeconds();
  //JupiterDataLog_M_D_Y_H_M_S
  /*name += "jupiterdatalog";
    name += String(month);
    name += "_";
    name += String(day);
    name += "_";
    name += String(yr);
    name += "_";
    name += String(hr);
    name += "_";
    name += String(minutes);
    name += "_";
    name += String(sec);
    name += ".txt";
    name = String(name);
    Serial.println(name);*/


  return name;
}

void printWiFiStatus(WiFiClient client) {
  // print the SSID of the network you're attached to:
  client.print("SSID: ");
  client.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  client.print("IP Address: ");
  client.println(ip);

  long rssi = WiFi.RSSI();
  client.print("signal strength (RSSI):");
  client.print(rssi);
  client.println(" dBm");
  client.print("<br>");
}

unsigned long sendNTPpacket(IPAddress& address) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123);  //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}
