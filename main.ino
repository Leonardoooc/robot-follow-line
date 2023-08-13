//----------------------------SENSOR DE COR---------------------------------------------
// referencia sensor de cor: https://www.hackster.io/millerman4487/arduino-color-recognition-71cd01
//Arduino pins:
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define sensorOut 6

#define TRIGGER A3
#define ECHO A2

#include <Ultrasonic.h>

Ultrasonic ultrasonic(A3, A2);


//Output from the sensor:
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

//Formatted color values:
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

//Values used for calibration
int redMin;
int redMax;
int greenMin;
int greenMax;
int blueMin;
int blueMax;

int preto = 0; int verde = 0; int branco = 0;

//----------------------------SENSOR INFRAVERMELHO--------------------------------------------- 
#define iv_esq A1
#define iv_dir A0
//---------------------------- MOTORES --------------------------------------------- 
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 9
#define ENA 11
#define ENB 10
int motor_speed = 130;
///////----------------------------MAIN VOIDS---------------------------------------------///////
void setup() {
 //Declarations:
 pinMode(S0, OUTPUT);
 pinMode(S1, OUTPUT);
 pinMode(S2, OUTPUT);
 pinMode(S3, OUTPUT); 
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENA, OUTPUT);
 pinMode(ENB, OUTPUT);
 pinMode(sensorOut, INPUT);
 pinMode(TRIGGER, OUTPUT);
 pinMode(ECHO, INPUT);

 digitalWrite(S0, HIGH);
 digitalWrite(S1, LOW);
 analogWrite(ENA, motor_speed);
 analogWrite(ENB, motor_speed);
 
 Serial.begin(9600);//begin serial communication
 delay(500);
 //calibrate();//calibrate sensor (look at serial monitor)
}

void loop() {
 //ColorSensor();
 lineFollow();
 //findBox();
}
///////----------------------------END MAIN VOIDS---------------------------------------------///////

int distance;

void findBox() {
  distance = ultrasonic.read();
  if (distance < 9) {
    Serial.print("Box found ");
    Serial.println(distance);
    moveRobot("box");
  }
}

//----------------------------SENSOR INFRAVERMELHO--------------------------------------------- 
void lineFollow()
{
  bool left_active  = (analogRead(iv_esq)>500);
  bool right_active = (analogRead(iv_dir)>500);

  distance = ultrasonic.read();
  Serial.println(distance);
  if (distance < 9) {
    Serial.print("Box found ");
    Serial.println(distance);
    moveRobot("box");
  }

  if (left_active && right_active) {
    Serial.println("encruzilhada");
  }
  else if (left_active)
  {
    Serial.println("esquerda");
  }
  else if (right_active)
  {
   Serial.println("direita");
  }
  else if(!right_active && !left_active)
  {
    Serial.println("frentee");
  }

  if (left_active && right_active) {
    moveRobot("crossroad");
  }
  else if(!right_active && !left_active)
  {
    moveRobot("forward");
  }  
  else if (right_active)
  {
    moveRobot("right");
  }
  else if (left_active)
  {
    moveRobot("left");
  }
}
//---------------------------- MOTORES --------------------------------------------- 
void moveRobot(String dir)
{
 //Gira o Motor A no sentido horario - direita tras IN1H IN2L
 //Gira o Motor A no sentido anti-horario - direita frente IN1L IN2H
 //Parar o Motor A - IN1H IN2H
 //Gira o Motor B no sentido horario - esquerda frente IN3L IN4H
 //Gira o Motor B no sentido anti-horario - esquerda tras IN3H IN4L
 //Parar o Motor B - IN3H IN4H
 if (dir == "forward")
 { 
    motorControl(1,0,100,1,0,100); 
 }
 else if(dir == "back")
 {
    motorControl(0,1,80,0,1,80); 
 }
 else if(dir == "left")
 {
    motorControl(0,1,130,1,0,150); 
 }  
 else if(dir == "right")
 {
    motorControl(1,0,150,0,1,130); 
 }
 else if(dir == "stop")
 {
    motorControl(1,1,0,1,1,0); 
 } else if(dir == "box")
 {
   bool firstLine;
   bool secondLine;
   //tras
   motorControl(0,1,120,0,1,120);
   delay(500);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //esquerda
   motorControl(0,1,130,1,0,150); 
   delay(450);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //frente
   motorControl(1,0,100,1,0,100); 
   delay(1300);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //direita
   motorControl(1,0,150,0,1,130); 
   delay(320);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //frente
   motorControl(1,0,100,1,0,105); 
   delay(2000);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //direita
   motorControl(1,0,150,0,1,130); 
   delay(350);
   //para
   motorControl(1,1,0,1,1,0); 
   delay(500);
   //frente
   motorControl(1,0,100,1,0,100); 
   delay(650);
   //esquerda
   motorControl(0,1,130,1,0,150); 
   delay(100);

  while(firstLine == false && secondLine == false) {
    motorControl(1,0,100,1,0,100);
    delay(100);
    motorControl(0,1,130,1,0,150); 
    delay(30);
    firstLine = (analogRead(iv_esq)>500);
    secondLine = (analogRead(iv_dir)>500);
  }
 }
 else if (dir == "crossroadd")
 {
   int verdeCount1 = 0;
   int verdeCount2 = 0;
   int verdeCount3 = 0;
   int verdeCount4 = 0;
   bool e1 = false;
   bool e2 = false;
   bool e3 = false;
   bool e4 = false;

   //E1 esquerda
   motorControl(0,1,130,1,0,150); 
   delay(350);
   motorControl(1,1,0,1,1,0); 

   for(int i = 0;i<30;i++) {
     color();
     calibragem();
     if (branco >= 45 && verde >= 40 && preto < 65){
      Serial.println(verde);
      Serial.println("Lendo verde");
      verdeCount1++;
      if (verdeCount1 > 4) {
        e1 = true;
      }
     }
   }
   delay(1000);
   //direita
   motorControl(1,0,150,0,1,130); 
   delay(350);
   motorControl(1,1,0,1,1,0); 
   delay(1000);

   //E2 direita
   motorControl(1,0,150,0,1,130); 
   delay(290);
   motorControl(1,1,0,1,1,0); 
   delay(1000);
   for(int i = 0;i<30;i++) {
     color();
     calibragem();
     if (branco >= 45 && verde >= 40 && preto < 65){
      Serial.println(verde);
      Serial.println("Lendo verde");
      verdeCount2++;
      if (verdeCount2 > 4) {
        e2 = true;
      }
     }
   }
   delay(1000);
   //esquerda
   motorControl(0,1,130,1,0,150); 
   delay(350);
   motorControl(1,1,0,1,1,0);
   delay(1000);

  //E3 frente
   motorControl(1,0,100,1,0,100); 
   delay(450);
   motorControl(1,1,0,1,1,0);
   delay(1000);
   //direita
   motorControl(1,0,150,0,1,130); 
   delay(350);
   motorControl(1,1,0,1,1,0); 
   for(int i = 0;i<30;i++) {
     color();
     calibragem();
     if (branco >= 45 && verde >= 40 && preto < 65){
      Serial.println(verde);
      Serial.println("Lendo verde");
      verdeCount3++;
      if (verdeCount3 > 4) {
        e3 = true;
      }
     }
   }
   delay(1000);
   //esquerda
   motorControl(0,1,130,1,0,150); 
   delay(350);
   motorControl(1,1,0,1,1,0);
   delay(1000);

   //E4 esquerda
   motorControl(0,1,130,1,0,150); 
   delay(350);
   motorControl(1,1,0,1,1,0);
   for(int i = 0;i<30;i++) {
     color();
     calibragem();
     if (branco >= 45 && verde >= 40 && preto < 65){
      Serial.println(verde);
      Serial.println("Lendo verde");
      verdeCount4++;
      if (verdeCount4 > 4) {
        e4 = true;
      }
     }
   }
   delay(1000);
   //direita
   motorControl(1,0,150,0,1,130); 
   delay(350);
   motorControl(1,1,0,1,1,0);
   delay(1000);
   motorControl(0,1,120,0,1,120);
   delay(550);
   motorControl(1,1,0,1,1,0);
   delay(7000);
   /*if (!e1 && !e2 && !e3 && !e4) {
     //frente
     motorControl(1,0,100,1,0,100); 
     delay(450);
   }
   if(e1 && !e2 && !e3 && !e4) {
     //esquerda
     motorControl(1,0,150,0,1,130); 
     delay(350);
   } */
 }
}

void motorControl (int b3, int b4, int speedB, int a1, int a2, int speedA)
{
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
  digitalWrite(IN1, a1);
  digitalWrite(IN2, a2);  
  digitalWrite(IN3, b3);
  digitalWrite(IN4, b4); 
}

//----------------------------SENSOR DE COR---------------------------------------------
void calibragem()
{
  
if (branco > 0 && branco < 45 && verde < 45 && preto < 45){
Serial.println(branco);
Serial.println("Lendo branco");
}

if (branco >= 45 && verde >= 40 && preto < 65){
Serial.println(verde);
Serial.println("Lendo verde");
}

if (branco > 60 && verde > 60 && preto >= 65){
Serial.println(preto);
Serial.println("Lendo preto");
}

}
//--------------------------------------------------------------------------------------------------------
void color()
{
//Rotina que le o valor das cores
digitalWrite(S2, LOW); digitalWrite(S3, LOW);

//count OUT, pverelho, RED
preto = pulseIn(sensorOut, digitalRead(sensorOut) == HIGH ? LOW : HIGH);
digitalWrite(S3, HIGH);

//count OUT, pBLUE, BLUE
branco = pulseIn(sensorOut, digitalRead(sensorOut) == HIGH ? LOW : HIGH);
digitalWrite(S2, HIGH);

//count OUT, pverde, verde
verde = pulseIn(sensorOut, digitalRead(sensorOut) == HIGH ? LOW : HIGH);
}
