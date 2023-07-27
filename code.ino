#include <TinyGPS++.h>                                  // Tiny GPS Plus Library
#include <SoftwareSerial.h>                             // Software Serial Library so we can use other Pins for communication with the GPS module
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial



const int trigPin = 5;
const int echoPin = 4;

//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

static const int RXPin = 12, TXPin = 13;                // Ublox 6m GPS module to pins 12 and 13
static const uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600

//const double Home_LAT = --.-----;                      // Your Home Latitude
//const double Home_LNG = --.-----;                      // Your Home Longitude
unsigned int move_index = 1;                            // fixed location for now
float spd;                                              //Variable  to store the speed
float sats;                                             //Variable to store no. of satellites response
String bearing;                                         //Variable to store orientation or direction of GPS

TinyGPSPlus gps;                                        // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                        // The serial connection to the GPS device
WidgetMap myMap(V0);                                    // V0 for vitrual pin of Map Widget
BlynkTimer timer;

char auth[] = "-dUnVFkh3XCjHRC3r2luuROKegFzrV0U";                   //Your Project authentication key
char ssid[] = "VIRUS";                              // Name of your WiFi network (HotSpot or Router name)
char pass[] = "EXTC2022";                              //Type your WiFi Password

void setup()
{  
  Serial.begin(9600);
 Blynk.begin(auth, ssid, pass);
  delay(1500);
  Serial.println("WiFi Connected!");
  delay(1500);

 // display.println("WiFi Connected!");
  //display.update();                                     // Update display
  delay(1500);                                          // Pause 1.5 seconds  
  ss.begin(GPSBaud);                                    // Set Software Serial Comm Speed to 9600    
 

  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

//  display.println("WiFi Connected!");
}



void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
      Blynk.virtualWrite(V4, "GPS ERROR");  // Value Display widget  on V4 if GPS not detected
  }
  
  }

void loop()
{  
  Blynk.run();
  timer.run();

  Serial.print("LAT: ");
  Serial.println(gps.location.lat(),5);
  

  Serial.print("LON: ");
  Serial.println(gps.location.lng(),5);
  Serial.println("\n");
 

  while(ss.available() > 0)
  {
    //skecth displays information every time a new sentence is correctly encoded
    //similar to Smartdelay function
    if (gps.encode(ss.read()))
     displayInfo(); 
    
  }
  

  
  smartDelay(500);                                      // Run Procedure smartDelay

  if (millis() > 5000 && gps.charsProcessed() < 10)
  Serial.println(F("No GPS data received: check wiring"));

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_VELOCITY/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance on the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);
  delay(1000);
    Blynk.virtualWrite(V6, distanceCm);  
     if(distanceCm <10) 
 Blynk.notify("Alert - WasteBin Full"); 
 //else 
 //Blynk.notify("Alert Tank Empty");
 
}

static void smartDelay(unsigned long ms)                // This custom version of delay() ensures that the gps object is being "fed".
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


void displayInfo()
{ 
  if (gps.location.isValid() ) 
  {    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 
    float longitude = (gps.location.lng()); 
    
    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V1, String(latitude, 6));   
    Blynk.virtualWrite(V2, String(longitude, 6));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();               //get speed
       Blynk.virtualWrite(V3, spd);
       
       sats = gps.satellites.value();    //get number of satellites
       Blynk.virtualWrite(V4, sats);

       bearing = gps.course.deg(); // get the direction
       Blynk.virtualWrite(V5, bearing);                   
  }
  
 Serial.println();
}
