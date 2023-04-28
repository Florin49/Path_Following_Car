#include <DHT.h>
#include <NewPing.h>
#include <QTRSensors.h>
//dht11 pins
#define DHTTYPE DHT11 
#define DHTPIN A0


#define MAX_DISTANCE 20

//ultra left pins
#define TRIGGER_PIN1  6
#define ECHO_PIN1     7


//ultra right pins 
#define TRIGGER_PIN2  5
#define ECHO_PIN2     4

///buzzer pin
#define BUZZ 2
///Buzzer variable

//Motors pins 
int in1=8;
int in2=9;
//int enA=0;

int in3=10;
int in4=11;
//int enB=12;

///Ultrsonics + dht declaration 
#define SENZOR1 A1
#define SENZOR2 A2
#define SENZOR3 A3
DHT dht(DHTPIN, DHTTYPE);


///Reflectance sensor
QTRSensors qtr;
const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];


void setup() {
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3}, SensorCount);
  qtr.setEmitterPin(2);
  
  pinMode(TRIGGER_PIN1,OUTPUT);
  pinMode(ECHO_PIN1,INPUT);  
  pinMode(TRIGGER_PIN2,OUTPUT);
  pinMode(ECHO_PIN2,INPUT); 
  pinMode(BUZZ,OUTPUT);
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  
  MonitorizarePozitie();
  afisareTH();
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(1500);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(1500);
 
}





void afisareTH(){
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(h) || isnan(t)){
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
 }
 Serial.print(F("Humidity: "));
 Serial.print(h);
 Serial.print(F("%  Temperature: "));
 Serial.print(t);
 Serial.print(F("°C "));
 Serial.println(F(""));
}


void MonitorizarePozitie(){
  digitalWrite(TRIGGER_PIN1, LOW);
  digitalWrite(TRIGGER_PIN2, LOW);
  delayMicroseconds(2);
  

  digitalWrite(TRIGGER_PIN1, HIGH);
  digitalWrite(TRIGGER_PIN2, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(TRIGGER_PIN1, LOW);
  digitalWrite(TRIGGER_PIN2, LOW);
  
// Reads the echoPin, returns the sound wave travel time in microseconds
  long duration1 = pulseIn(ECHO_PIN1, HIGH);
  int distance1= duration1*0.034/2;
  int safetyDistance1 = distance1;

  long duration2 = pulseIn(ECHO_PIN2, HIGH);
  int distance2= duration2*0.034/2;
  int safetyDistance2 = distance2;
  
  if (safetyDistance1 <=100  || safetyDistance2<=100){
  digitalWrite(BUZZ, HIGH);
  
}
else{
  digitalWrite(BUZZ, LOW);
  
}

// Prints the distance on the Serial Monitor
Serial.print("Distances: ");
Serial.print(distance1);
Serial.print(" ");
Serial.print(distance2);
Serial.println("");
}


void LineDetector(){
  unsigned int sensors[3];
  int position=0;
}
