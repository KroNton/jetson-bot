#include <Arduino.h>


// Pin definitions for motor control
const int motorFL1 = 2;       // Front-left motor positive
const int motorFL2 = 3;        // Front-left motor negative
const int motorFL_Enable = 4; // Front-left motor enable pin

const int motorBL1 = 8;       // Back-left motor positive
const int motorBL2 = 7;        // Back-left motor negative
const int motorBL_Enable = 9; // Back-left motor enable pin


const int motorBR1 = 14;       // Back-right motor positive
const int motorBR2 = 13;       // Back-right motor negative
const int motorBR_Enable = 15; // Back-right motor enable pin

const int motorFR1 = 20;       // Front-right motor positive
const int motorFR2 = 21;       // Front-right motor negative
const int motorFR_Enable = 22; // Front-right motor enable pin





double w_fl;
double w_fr;
double w_rl;
double w_rr;

double Vx = 0.0;
double Vy = 0.0;
double Wz = 0.0;

// double Vx_ = 0;
// double Vy_ = 0;
// double Wz_ = 0;

double pwm_fl = 0;
double pwm_fr = 0;
double pwm_rl = 0;
double pwm_rr = 0;

void motorsSetup(){
  // Set motor control pins as outputs
  pinMode(motorFL1, OUTPUT);
  pinMode(motorFL2, OUTPUT);
  pinMode(motorFL_Enable, OUTPUT);

  pinMode(motorFR1, OUTPUT);
  pinMode(motorFR2, OUTPUT);
  pinMode(motorFR_Enable, OUTPUT);

  pinMode(motorBL1, OUTPUT);
  pinMode(motorBL2, OUTPUT);
  pinMode(motorBL_Enable, OUTPUT);

  pinMode(motorBR1, OUTPUT);
  pinMode(motorBR2, OUTPUT);
  pinMode(motorBR_Enable, OUTPUT);


}



// Function to move the robot forward
void straightAhead(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}
void straightBack(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}

void sideWay(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}


void sideWayInv(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}

void turnRound(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}
void turnRoundInv(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, v_fl);
    analogWrite(motorFR_Enable, v_fr);
    analogWrite(motorBL_Enable, v_rl);
    analogWrite(motorBR_Enable, v_rr);

    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}

void OFF()
{
    // Set enable pins HIGH to enable the motors
    analogWrite(motorFL_Enable, 0);
    analogWrite(motorFR_Enable, 0);
    analogWrite(motorBL_Enable, 0);
    analogWrite(motorBR_Enable, 0);
}