#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //RX = 2 ; TX = 3

//Variable pour stocker données des 3 capteurs IR
bool valeurCapteur[3];

//Variables pour le régulateur PID
float err;
float error = 0;
float pre_error = 0;
float Kp;
float Kd;
float Ki;
float resultPIDF;
float sum_error = 0;
int resultPID;
int rightSpeed,leftSpeed;
int baseSpeed = 150;
const int maxSpeed = 255;
int vd, vg;

// Driver Moteur
const int ENA = 5;
const int IN1 = 4;
const int IN2 = 6;
const int ENB = 9;
const int IN3 = 7;
const int IN4 = 8;
////////////////////////////////
////////////////////////////////


void setup(){
//Driver Moteur
pinMode(ENA,OUTPUT);
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(ENB,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);

//Les capteurs IR
pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);

mySerial.begin(9600);

//
//
//
while (mySerial.available() == 0)
Kp = mySerial.parseFloat();
mySerial.print("KP = ");mySerial.println(Kp);mySerial.flush(); 
while (mySerial.available()) mySerial.read(); 
//
while (mySerial.available() == 0)
Ki = mySerial.parseFloat();
mySerial.print("Ki = ");mySerial.println(Ki);mySerial.flush();
while (mySerial.available()) mySerial.read();
//
while (mySerial.available() == 0)
Kd = mySerial.parseFloat();
mySerial.print("Kd = ");mySerial.println(Kd);mySerial.flush();
while (mySerial.available()) mySerial.read();
//
while (mySerial.available() == 0)
err = mySerial.parseFloat();
mySerial.print("err = ");mySerial.println(err);mySerial.flush();
while (mySerial.available()) mySerial.read();
//
while (mySerial.available() == 0)
baseSpeed = mySerial.parseInt();
mySerial.print("baseSpeed = ");mySerial.println(baseSpeed);mySerial.flush();
while (mySerial.available()) mySerial.read();
//

delay(1000);
mySerial.print("The Robot is ready!!");
}

void loop(){
	
  suiveur();
}



//Stop les deux moteurs
void stopM(){
	digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
	digitalWrite(ENA,LOW);
	digitalWrite(ENB,LOW);
}

//Aller tout droit
void avant() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
}

//Fonction pour traiter les commandes négatives envoyées au moteur
void moteur(int vd, int vg) {

  if (vd > 0 && vg > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, vg);
    analogWrite(ENB, vd);
    //Serial.print("tout droit");
  }
  else if (vd < 0 && vg > 0) {
    vd *= -1;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, vg);
    analogWrite(ENB, vd);
    //Serial.println("tourner gauche");
  }
  else if (vd > 0 && vg < 0) {
    vg *= -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, vg);
    analogWrite(ENB, vd);
    //Serial.println("tourner droite");
  }
}

//
void suiveur() {
  /////////////////////////////////////////
  //    Lire les capteurs              ////
  //////////////////////////////////////////
  /**/valeurCapteur[0] = digitalRead(A0); //
  /**/valeurCapteur[1] = digitalRead(A1); //
  /**/valeurCapteur[2] = digitalRead(A2); //
  /////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 									                  Erreur lu sur les 3 capteurs                             						//
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if      (valeurCapteur[0] == 0 && valeurCapteur[1] == 1 && valeurCapteur[2] == 0 ) {
    error = 0;
  }
  else if (valeurCapteur[0] == 0 && valeurCapteur[1] == 1 && valeurCapteur[2] == 1 ) {
    error = 1;
  }
  else if (valeurCapteur[0] == 0 && valeurCapteur[1] == 0 && valeurCapteur[2] == 1 ) {
    error = err;
  }
  else if (valeurCapteur[0] == 1 && valeurCapteur[1] == 1 && valeurCapteur[2] == 0 ) {
    error = -1;
  }
  else if (valeurCapteur[0] == 1 && valeurCapteur[1] == 0 && valeurCapteur[2] == 0 ) {
    error = -err;
  }
  else if (valeurCapteur[0] == 1 && valeurCapteur[1] == 1 && valeurCapteur[2] == 1 ) {
    error = 0;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  resultPIDF = Kp * error + Kd * (error - pre_error) + Ki * (sum_error);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**/   leftSpeed = baseSpeed + resultPIDF;
  /**/   rightSpeed = baseSpeed - resultPIDF;
  ////////////////////////////////////////////////////////////////////
  //Ne pas dépasser la valeur max (255) envoyer au moteur par PWM
  if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
  if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;

  if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
  if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;
  ////////////////////////////////////////////////////////////////////
  moteur(rightSpeed, leftSpeed);
  
  pre_error = error;
  sum_error += error;
  
  delay(1);
}
