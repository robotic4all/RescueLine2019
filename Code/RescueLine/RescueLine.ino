/*	#############################################
	# 			ROBÔ DE RESGATE (OBR)			#
	#			 EQUIPE ROBOTIC4ALL				#
	#					2019					#
	#############################################  */

/* INCLUINDO BIBLIOTECAS */
#include <QTRSensors.h>
#include <MotorController.h>
#include <Servo.h>
#include <SensorColor.h>
#include <UltraDistance.h>

/* ARRAY SENSOR */
#define NUM_SENSORS             6
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             2

QTRSensorsAnalog arraySensors((unsigned char[]) {A0, A1, A2, A3, A4, A5}
	,NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

unsigned int position = 0; 

/* CONTROLADOR PID */
float Kp = 0;  
float Ki = 0;
float Kd = 0;

int setPoint = 2500;

int P = 0,
	lastP = 0,
	I = 0, 
	D = 0;

int PIDValue = 0;

int	error6 = 0,
	error5 = 0,
	error4 = 0,
	error3 = 0,
	error2 = 0,
	error1 = 0;

int rightMotorSpeed = 0, 
	leftMotorSpeed 	= 0, 
	baseSpeed 		= 50;

int speedMax = 100,
	speedMin =   0;

/* BUZZER */
#define buz  21

/* LED */
#define led  45

/* MOTORES */
#define motorRightSpeed 	3
#define motorLeftA 			5
#define motorLeftB  		4
#define motorRightA	  		8
#define motorRightB	  		7
#define motorRightSpeed		9

MotorController motors(motorLeftA, motorLeftB, motorRightSpeed,
						motorRightA, motorRightB, motorRightSpeed);

/* SERVO 1 */
#define servoPin1 22
Servo servo1;

/* SERVO 2 */
#define servoPin2 23
Servo servo2;

/* SENSORES DE VERDE */
#define minRed 		-11000
#define maxRed 		-4000
#define minGreen 	-4000
#define maxGreen 	-2000
#define minBlue 	-10000
#define maxBlue 	-4000

/* SENSOR DO VERDE - TCS230 - ESQUERDA */
#define senLeftS0 	11
#define senLeftS1 	12
#define senLeftS2 	13
#define senLeftS3 	14
#define senLeftOut 	 9

SensorColor senLeftGreen(	senLeftS0, 
							senLeftS1, 
							senLeftS2, 
							senLeftS3, 
							senLeftOut,
						 	minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue);

/* SENSOR DO VERDE - TCS230 - DIREITA */
#define senRightS0 	17
#define senRightS1 	20
#define senRightS2 	21
#define senRightS3 	22
#define senRightOut 15

SensorColor senRightGreen(	senRightS0, 
							senRightS1, 
							senRightS2, 
							senRightS3, 
							senRightOut,
						 	minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue );

/* ULTRA FRENTE - - SF = SENSOR FRENTE*/
#define trigSF 26
#define echoSF 27

UltraDistance ultraFront(trigSF, echoSF);

/* ULTRA TRÁS - ST = SENSOR TRÁS*/
#define trigSB 28
#define echoSB 29

UltraDistance ultraBack(trigSB, echoSB);

/* ULTRA ESQUERDA - SL = SENSOR LEFT*/
#define trigSL 30
#define echoSL 31

UltraDistance ultraEsquerda(trigSL, echoSL);

/* ULTRA DIREITA - SD = SENSOR RIGHT*/
#define trigSR 32
#define echoSR 33

UltraDistance ultraDireita(trigSR, echoSR);

float distanceDivert = 4;

/* SENSOR SHARP - FRENTE */
#define sharpFront A8

void setup(void){
	Serial.begin (9600);

	/* BUZZER */
	pinMode(buz, OUTPUT);

	/* SERVO 1 */
	servo1.attach(servoPin1);

	/* SERVO 2 */
	servo2.attach(servoPin2);

	/* MOTORES - PARADOS */
	motors.stop();

	/* ARRAY SENSOR - CALIBRAÇÃO */
	calibra();
}

void calibra(){
	delay(500);

	Serial.println("\n</Calibração Iniciada>");

	for (int i=0; i<70; i++){
		Serial.println("	Calibrando..");
		digitalWrite(led, HIGH); 
		delay(20);
		arraySensors.calibrate();
		digitalWrite(led, LOW);  
		delay(20);
	}

	delay(3000);

	Serial.println("</Calibração Terminada>");
}

void readArraySensors(){
	Serial.print("\n</Leitura Array Sensor = ");
	
	position = arraySensors.readLine(sensorValues, true, false);
	
	Serial.println(position);
	Serial.println(" >");
}

void calculatePID(){
	Serial.println("\n</Inicio Calculo PID>");
	P 	= ((int)position) - setPoint;
	Serial.print("	P = ");
	Serial.println(P);
	
	D 	= P - lastP; 
	Serial.print("	D = ");
	Serial.println(D);
	
	I 	= error1 + error2 + error3 + error4 + error5 + error6;
	Serial.print("	I = ");
	Serial.println(I);

	lastP 	= P;

	error6 = error5;
	error5 = error4;  
	error4 = error3;  
	error3 = error2;
	error2 = error1;
	error1 = P;

	PIDValue = (P * Kp) + (D * Kd) + (I * Ki);

	if(PIDValue < (setPoint * -1)){
		PIDValue = (setPoint * -1);
	}
	if(PIDValue > setPoint){
		PIDValue = setPoint;
	}

	PIDValue = map(PIDValue, -2500, 2500, -50, 50);

	Serial.print("\n	PIDValuue = ");
	Serial.println(PIDValue);
	Serial.println("</Fim Calculo PID>");

}

void motor(){
	rightMotorSpeed = baseSpeed - PIDValue;
	leftMotorSpeed = baseSpeed + PIDValue;

	if(rightMotorSpeed >= speedMax) rightMotorSpeed = speedMax;
	if(rightMotorSpeed <= speedMin) rightMotorSpeed = speedMin;

	if(leftMotorSpeed >= speedMax) leftMotorSpeed = speedMax;
	if(leftMotorSpeed <= speedMin) leftMotorSpeed = speedMin;

	rightMotorSpeed = map(rightMotorSpeed, 0, 100, 0, 255);
	leftMotorSpeed 	= map(leftMotorSpeed,  0, 100, 0, 255);

	Serial.println("\n</Velocidade Motores>");
	Serial.print("	Esquerdo = ");
	Serial.println(leftMotorSpeed);
	Serial.print("	Direito = ");
	Serial.println(rightMotorSpeed);
	Serial.println(">");

	motors.forward(leftMotorSpeed,rightMotorSpeed); //Para Frente
}

void som(int freq, int temp){
	tone(buz, freq);
	delay(temp);
	noTone(buz);
	delay(temp);
}

float distanceSharp(int sensor){

	int sensorValue = analogRead(sensor);
	float cmValue = (6762 / (sensorValue - 9)) - 4;


	return cmValue;
}

void followLine(){
	readArraySensors(); 	//Leitura

	calculatePID(); 		//Processamento 

	motor(); 				//Retorno
}

void checkGreen(){
	if(senRightGreen.isGreen() && senLeftGreen.isGreen()){
		Serial.println("</ BECO DETECTADO >");
		//Beco
	} 
	else if(!senRightGreen.isGreen() && senLeftGreen.isGreen()){
		Serial.println("</ VERDE DETECTADO - ESQUERDA >");
		//Virar Esquerda
	}
	else if(senRightGreen.isGreen() && !senLeftGreen.isGreen()){
		Serial.println("</ VERDE DETECTADO - direita >");
		//Virar Direita
	}
}

void obstacle(){
	if((ultraFront.whatDistance() >= distanceDivert) && 
	    distanceSharp(sharpFront >= distanceDivert)){
		Serial.println("</INICIO DESVIO DE OBSTACULO>");
		//Desviando
		Serial.println("</FIM DESVIO DE OBSTACULO>");
	}
}

void rescueLine(){
	checkGreen();

	followLine();

	obstacle();
}

void loop(void){
	rescueLine();
}
