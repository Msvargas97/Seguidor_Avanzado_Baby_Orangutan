#define F_CPU 20000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <SoftwareSerial.h>
#include <PololuQTRSensors.h>

#define TOPE 15
#define NUM_SENSORS   6    // number of sensors used
#define RxD 12 // Pin TX HC-05
#define TxD 13 // Pin RX HC-05
#define RED 9
#define BLUE 8
#define GREEN 10
#define pintaR 7
#define pintaL A0
SoftwareSerial BTSerial(RxD, TxD);
//###########VARIABLES#############
//  7 y A0
unsigned char qtr_pins[NUM_SENSORS]={4,2,0,A3,A2,A1};//Pines de sensores
int max =123;
unsigned int setPoint=2500;
char desviacion;
float FACTOR_SPEED = 0.42;
int proportional = 0;      // proporcional
int position;
int derivative;
int power_difference = 0;  // velocidad diferencial
int last_proportional=0;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;
unsigned int cruces,BIF;
boolean Flag=false,Flag2=false;
    unsigned int sensorValues[NUM_SENSORS];
//########Espacio para constantes#########
//const byte BLUE = 8,GREEN=10 ,RED=9;
// Constantes Proporcional y Derivativa
float KP = 0.0614;
float KD = 3.7124;
//#####FUNCIONES
void setup_timers(){
TCCR0A = TCCR2A = 0xF3;
TCCR0B = TCCR2B = 0x02;
// initialize all PWMs to 0% duty cycle (braking)   
OCR0A = OCR0B = OCR2A = OCR2B = 0;
}
void setMotorLeft(int speed)
{
  unsigned char reverse = 0;
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
    if (reverse)
  {
    OCR2B = 0;    // hold one driver input high
    OCR2A = speed;  // pwm the other input
  }
  else  // forward
  {
    OCR2B = speed;  // pwm one driver input
    OCR2A = 0;    // hold the other driver input high
  }
}
// función Velocidad Motor Izquierdo
void setMotorRight(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
  if (reverse)
  {
    OCR0B = 0;    // hold one driver input high
    OCR0A = speed;  // pwm the other input
  }
  else  // forward
  {
    OCR0B = speed;  // pwm one driver input
    OCR0A = 0;    // hold the other driver input high
  }

}

// función Velocidad Motores
void setMotors(int left, int right)
{
 setMotorRight(right); 
 setMotorLeft(left);
}
void setMotors2(int left, int right)
{
if(left==0 && right == max){
setMotorLeft(-(max*FACTOR_SPEED));setMotorRight(max); if(proportional >=3500)delayMicroseconds(10*8);
}else if(right==0 && left == max){
setMotorLeft(max);setMotorRight(-(max*FACTOR_SPEED)); if(proportional <=-3500)delayMicroseconds(10*8);
}else if (left > TOPE || right > TOPE){
   if(left>TOPE){setMotorLeft(left);}
   if(right>5){setMotorRight(right);}
}else{ 
  if(right<=TOPE){setMotorRight(-TOPE);  }
  if(left<=TOPE){setMotorLeft(-TOPE);}
}
}
void readLine() {
  position = qtr_read_line(sensorValues, QTR_EMITTERS_ON); //0 para linea
  //negra, 1 para linea blanca    // Obtiene la posición de la linea
  // Aquí no estamos interesados ​​en los valores
  // individuales de cada sensor
  // El término proporcional debe ser 0 cuando estamos en línea
  proportional = ((int)position) - setPoint;
  derivative = proportional - last_proportional;
  // Recordando la última posición
  last_proportional = proportional;

}
void rgbColor(boolean R,boolean G, boolean B){
digitalWrite(RED,R);
digitalWrite(GREEN,G);
digitalWrite(BLUE,B);
}
void setup() {
qtr_rc_init(qtr_pins,6,2000,1);//Inicializar sensores
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(3,OUTPUT);
pinMode(11,OUTPUT);
pinMode(RED,OUTPUT);
pinMode(BLUE,OUTPUT);
pinMode(GREEN,OUTPUT);
pinMode(pintaR,INPUT);
pinMode(pintaL,INPUT);
BTSerial.flush();//Limpiar datos
BTSerial.begin(38400); //Inicializar BT
BTSerial.flush();

inputString.reserve(200);//Reservar 200 bytes para almacenar datos BT
BTSerial.println("Odin2.0 Reborn");
BTSerial.println("Powered By Michael Vargas");
BTSerial.println("--------------------------");
qtr_emitters_off();

 setup_timers();
 rgbColor(1,0,0);
 while(true){
  /* serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
    cont++;
   if(cont==1){ KD=inputString.toFloat();
   BTSerial.print("KD=");
    BTSerial.println(KD);}else if(cont==2){
   KP=inputString.toFloat();
   BTSerial.print("KP="); BTSerial.println(KP);
   max=map(analog.readTrimpot(),0,1023,0,255);
   BTSerial.print("Velocidad="); BTSerial.println(max);
   cont=0;
  
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }*/
 
   if(analogRead(A6) < 100){

   setMotors(0,0);
    boolean changer;
    for ( int i = 0; i<80; i++)
    {
       changer=!changer;
      rgbColor(changer,0,changer);
       if (i < 17 ){  setMotors(0, 0);
       } else if( i<35){
       setMotors(0,0);
      }else if(i<50 ){
         setMotors(0,0);
      }else{
       setMotors(0,0);
      }
      qtr_calibrate(QTR_EMITTERS_ON);
       // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20*8);
    }
    setMotors(0,0);
   break;
   }
  } 
  qtr_emitters_off();
    while(true){ 
    if(analogRead(A6) < 100)break;
  }
    max=analogRead(A7)/4;
qtr_emitters_on();
   delay(500*8);
  rgbColor(0,0,0);
};

void loop() {
  boolean SL = digitalRead(pintaL);
  boolean SR = digitalRead(pintaR);
  if(SL && SR){
    if(!Flag){
     cruces++;
     if(cruces==1 || cruces==3 || cruces==6 || cruces==8 || cruces==16){
        BIF =1;
     }
     setPoint=2500;
     Flag=true; 
    }
  }else if(!SL && !SR){
      Flag=false;  
  }

switch(cruces){
 case 1:
 if(!SL && SR && BIF ==1) {
   setPoint=3500;
 rgbColor(0,0,0);
 rgbColor(1,0,0);
 desviacion='R';
 Flag2=false;
 }
 if(desviacion == 'R' && SL && !SR && BIF==1){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=2;
 }
 if(!SL && !SR && BIF==2){
 Flag2=true;
 }
 if(SL && !SR && Flag2==true) {
 setPoint=1500;
 rgbColor(0,0,0);
 rgbColor(0,0,1);
 desviacion='L';
 }
 if(desviacion == 'L' && SR && BIF==2){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=0;
 }
break; 
case 2:
case 3:
if(!SL && SR && BIF ==1) {
   setPoint=3500;
 rgbColor(0,0,0);
 rgbColor(1,0,0);
 desviacion='R';
 }
 if(desviacion == 'R' && SL && BIF==1){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=2;
 }
break;
case 4:
case 5:
case 6:
if(SL && !SR && BIF ==1) {
   setPoint=1500;
 rgbColor(0,0,0);
 rgbColor(1,0,0);
 desviacion='L';
 }
 if(desviacion == 'L' && SR && BIF==1){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=2;
 }
break;
case 7:
case 8:
if(!SL && SR && BIF ==1) {
   setPoint=4800;
 rgbColor(0,0,0);
 rgbColor(1,0,0);
 desviacion='R';
 }
 if(desviacion == 'R' && SL && BIF==1){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=2;
 }
break;
case 9:
case 10:
case 11:
case 12:
case 13:
case 14:
case 15:
case 16:
if(SL && !SR && BIF ==1) {
   setPoint=1500;
 rgbColor(0,0,0);
 rgbColor(1,0,0);
 desviacion='L';
 }
 if(desviacion == 'L' && SR && BIF==1){
 setPoint=2500;
 rgbColor(0,0,0);
 rgbColor(0,1,0); 
 BIF=2;
 }
break;
case 19:
rgbColor(1,1,1);
cruces=0;
break;
default:
setPoint=2500;
break;
  
}
max=analogRead(A7)/4;
  readLine();
  // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ].
  // Si es un número positivo, el robot gira a la [ derecha ]
  // Si es un número negativo, el robot gira a la [ izquierda ]
  //  y la magnitud del número determina el ángulo de giro.
  int power_difference = ( proportional * KP ) + ( derivative * KD );
  // Si velocidad diferencial es mayor a la posible tanto positiva como negativa,
  // asignar la máxima permitida
  if ( power_difference > max ) power_difference = max;
  else if ( power_difference < -max ) power_difference = -max;
  

  // Asignar velocidad calculada en el poder diferencial de los motores
  ( power_difference < 0 ) ? setMotors2(max, max + power_difference) : setMotors2(max - power_difference, max);
};

//########Espacio para funciones ########
void serialEvent() {
  while (BTSerial.available()) {
    // get the new byte:
    char inChar = (char)BTSerial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


