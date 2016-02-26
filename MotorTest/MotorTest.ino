#include <stdint.h>
#include <cmath>

//#include "Queue.h"
//
//int pinEncRightA = 6;
//int pinEncRightB = 7;
//int pinEncLeftA = 4;
//int pinEncLeftB = 5;
//int pinMotRightPWM = 11;
//int pinMotRightFwd = 12;
//int pinMotRightBck = 13;
//int pinMotLeftPWM = 10;
//int pinMotLeftFwd = 9;
//int pinMotLeftBck = 8;
//
//
//const float L = 18.8;  // Diameter of the robot
//const float rw = 9 / 2.0; // Radius wheel
//const int COUNTS_PER_REV = 1600;
//
//float EncR = 0, EncL = 0; // Encoder counts
//// Throttle left, right (-255, 255) 
//float TL = 0, TR = 0; 
//
//#define sign(a) (((a) < 0) ? -1 : ((a) > 0))
//
//// Command queue
//CQueue commandQueue;
//
//void RightAR() {
//  if (digitalRead(pinEncRightB) == LOW)
//    EncR--;
//  else
//    EncR++;
//}
//
//void RightBR() {
//  if (digitalRead(pinEncRightA) == HIGH)
//    EncR--;
//  else
//    EncR++;
//}
//
//
//
//void LeftAR() {
//  if (digitalRead(pinEncLeftB) == LOW)
//    EncL--;
//  else
//    EncL++;
//}
//
//void LeftBR() {
//  if (digitalRead(pinEncLeftA) == HIGH)
//    EncL--;
//  else
//    EncL++;
//}
//
//
//void enqueueRotateAngle(float dtheta) {
//  float tc = dtheta * L / (4 * M_PI * rw) * COUNTS_PER_REV;
//  commandQueue.push(-tc, tc, false);
//}
//
//void enqueueForward(float dist) {
//  float tc = dist / (2 * M_PI * rw) * COUNTS_PER_REV;
//  commandQueue.push(tc, tc, false);
//}
//
//void setTarget(float dx, float dy) {
//  float dtheta = atan2(dy, dx);
//  float dist = sqrt(dx * dx + dy * dy);
//  enqueueRotateAngle(dtheta);
//  enqueueForward(dist);
//}
//
//
//void setThrottle(float TL, float TR) {
//  if (TL >= 0) {
//    digitalWrite(pinMotLeftFwd, HIGH);
//    digitalWrite(pinMotLeftBck, LOW);
//  } else {
//    digitalWrite(pinMotLeftFwd, LOW);
//    digitalWrite(pinMotLeftBck, HIGH);
//  }
//
//  if (TR >= 0) {
//    digitalWrite(pinMotRightFwd, HIGH);
//    digitalWrite(pinMotRightBck, LOW);
//  }
//  else {
//    digitalWrite(pinMotRightFwd, LOW);
//    digitalWrite(pinMotRightBck, HIGH);
//  }
//  analogWrite(pinMotLeftPWM, abs(TL));
//  analogWrite(pinMotRightPWM, abs(TR));
//}
//
//void processQueue() {
//  if (!commandQueue.empty()) {
//    CQNode *E = commandQueue.first;
//    if (!E->processing) {
//      EncL = 0;
//      EncR = 0;
//      E->processing = true;
//      TL = 255 * sign(E->CL);
//      TR = 255 * sign(E->CR);
//      setThrottle(TL, TR);
//    }
//    else {
//      
//      float dl = -EncL - E->CL;
//      float dr = EncR - E->CR;
//      if (sqrt(dr * dr + dl * dl) < 50 || abs(EncR) > abs(E->CR) || abs(EncL) > abs(E->CL)) {
//        TL = 255 * sign(E->CL);
//        TR = 255 * sign(E->CR);
//        setThrottle(-TL, -TR);
//        delay(40);
//        setThrottle(0, 0);
//        delay(1000);
//        Serial.print(-EncL);
//        Serial.print(" ");
//        Serial.print(EncR);
//        Serial.print(" ");
//        Serial.print(E->CL);
//        Serial.print(" ");
//        Serial.println(E->CR);
//        commandQueue.pop();
//        return;
//      }
//
//      float error = -EncL * sign(E->CL) - EncR * sign(E->CR);
//
//      float Kp = 0.000001;
//      float adjust = Kp * error;
//
//      TL -= adjust * sign(E->CL);
//      TR += adjust * sign(E->CR);
//      TL = constrain(TL, -255, 255);
//      TR = constrain(TR, -255, 255);
//      setThrottle(TL, TR);
//
//    }
//  }
//}
//
//void setup() {
//  // initialize digital pin 13 as an output.
//  pinMode(pinMotRightFwd, OUTPUT);
//  pinMode(pinMotRightBck, OUTPUT);
//  pinMode(pinMotLeftFwd, OUTPUT);
//  pinMode(pinMotLeftBck, OUTPUT);
//  pinMode(pinEncRightB, INPUT);
//  pinMode(pinEncRightA, INPUT);
//
//  attachInterrupt(digitalPinToInterrupt(pinEncRightA), RightAR, RISING);
//  attachInterrupt(digitalPinToInterrupt(pinEncRightB), RightBR, RISING);
//
//  attachInterrupt(digitalPinToInterrupt(pinEncLeftA), LeftAR, RISING);
//  attachInterrupt(digitalPinToInterrupt(pinEncLeftB), LeftBR, RISING);
//
//  Serial.begin(57600);
//  delay(3000);
//  //setTarget(30, 0);
//  //enqueueRotateAngle(M_PI/ 2);
//  enqueueForward(100);
//  enqueueRotateAngle(M_PI);
//  enqueueForward(100);
//  /*delay(410);
//  setThrottle(0, 0);
//  delay(1000);
//  Serial.print(-EncL);
//  Serial.print(" ");
//  Serial.println(EncR);*/
//}
//
//// the loop function runs over and over again forever
//void loop() { 
//  processQueue(); 
//  //delay(1000);
//
// //setThrottle(250, -250);
//}

#define sign(a) (((a) < 0) ? -1 : ((a) > 0))

int pinEncLeftA = 6;
int pinEncLeftB = 7;
int pinEncRightA = 4;
int pinEncRightB = 5;
int pinMotRightPWM = 13; // or 8
int pinMotRightFwd = 11; 
int pinMotRightBck = 12; 
int pinMotLeftPWM = 8; // or 13
int pinMotLeftFwd = 9;
int pinMotLeftBck = 10;

float EncR = 0, EncL = 0; // Encoder counts

void LeftAR() { if (digitalRead(pinEncLeftB) == LOW)  EncL--; else EncL++; }
void LeftBR() { if (digitalRead(pinEncLeftA) == HIGH)  EncL--; else EncL++; }
void RightAR() { if (digitalRead(pinEncRightB) == LOW) EncR--; else EncR++; }
void RightBR() { if (digitalRead(pinEncRightA) == HIGH) EncR--; else EncR++; }


void setThrottle(float TL, float TR) {
  if (TL >= 0) {
    digitalWrite(pinMotLeftFwd, HIGH);
    digitalWrite(pinMotLeftBck, LOW);
  } else {
    digitalWrite(pinMotLeftFwd, LOW);
    digitalWrite(pinMotLeftBck, HIGH);
  }

  if (TR >= 0) {
    digitalWrite(pinMotRightFwd, HIGH);
    digitalWrite(pinMotRightBck, LOW);
  }
  else {
    digitalWrite(pinMotRightFwd, LOW);
    digitalWrite(pinMotRightBck, HIGH);
  }
  analogWrite(pinMotLeftPWM, abs(TL));
  analogWrite(pinMotRightPWM, abs(TR));
}

void setup() {
  pinMode(pinEncLeftA, INPUT);
  pinMode(pinEncLeftB, INPUT);
  pinMode(pinEncRightB, INPUT);
  pinMode(pinEncRightA, INPUT);
  
  pinMode(pinMotRightPWM, OUTPUT);
  pinMode(pinMotRightFwd, OUTPUT);
  pinMode(pinMotRightBck, OUTPUT);
  pinMode(pinMotLeftPWM, OUTPUT);
  pinMode(pinMotLeftFwd, OUTPUT);
  pinMode(pinMotLeftBck, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pinEncLeftA), LeftAR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncLeftB), LeftBR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncRightA), RightAR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncRightB), RightBR, RISING);
  Serial.begin(57600);
  Serial1.begin(57600);
}

void resetCounts() {
  EncR = 0;
  EncL = 0;
}

void moveToCount(int power, int CL, int CR) {
  int sl = sign(CL);
  int sr = sign(CR);
  int DD = (power - 80) * 0.6 + 20;
  int TT = (power - 80) * 0.26 + 24;
  setThrottle(power * sl, power * sr);
  
  while (abs(EncR) < abs(CL) - DD || abs(EncL) < abs(CR) - DD) {
    float De = (abs(EncL) - abs(EncR));
    float DPR = abs(EncR) - (abs(CL) - DD);
    float DPL = abs(EncL) - (abs(CR) - DD);
    float DP = sqrt(DPR * DPR + DPL * DPL);
    float kp = DP * 0.005;
 //   if (DP < 300) kp = 0;
    if (kp > 1)  kp = 1;
    if (De != 0) {
      float kd = 1 / (fabs(De) * 0.2);
      if (kd > 1) kd = 1;
      if (De > 0)
        setThrottle(power * sl * kd * kp, power * sr * kp);
      else
        setThrottle(power * sl * kp, power * sr * kd * kp);
    }
    delay(5);
  }
  setThrottle(-80 * sl, -80 * sr);
  delay(TT);
  setThrottle(0, 0);
  float De = (abs(EncL) - abs(EncR));
  Serial.print(EncL);
  Serial.print(" ");
  Serial.print(EncR);
  Serial.print(" - ");
  Serial.print(CL);
  Serial.print(" ");
  Serial.print(CR);
  Serial.print(" - ");
  Serial.print(abs(EncR) - abs(CL));
  Serial.print(" ");
  Serial.print(abs(EncL) - abs(CR));
  Serial.print(" - ");
  Serial.println(De);
  delay(300);


}

const float MIN_ANGLE = 5 * M_PI / 180.0f;
const float L = 205;  // Diameter of the robot
const float dw = 90; // Diameter of the wheel
const int COUNTS_PER_REV = 1600;

void moveToPos(int power, float dx, float dy) {
  float alpha = atan2(dx, dy);
  if (fabs(alpha) < MIN_ANGLE) {
    float d = sqrt(dx * dx + dy * dy);
    float c = d / (M_PI * dw) * COUNTS_PER_REV;
    resetCounts();
    moveToCount(power, c, c);
  }
  else {
    float dr = fabs(alpha) * L / 2;
    float cr = dr / ( dw * M_PI ) * COUNTS_PER_REV;
    float lcr = -cr;
    float rcr = cr;
    if (alpha > 0){
      lcr = cr;
      rcr = -cr;
    }
    resetCounts();
    moveToCount(power, lcr, rcr);

    float d = sqrt(dx * dx + dy * dy);
    float c = d / (M_PI * 90) * COUNTS_PER_REV;
    resetCounts();
    moveToCount(power, c, c);
  }


}


void loop1() {
  int n = Serial1.available();
  if (n > 5) {
    // Power: 2 byte 0..255
    // Dx: 2 bytes in mm
    // Dy: 2 bytes in mm
    int16_t v[3];
    Serial1.readBytes((char*)v, 6);
    moveToPos(v[0], v[1], v[2]);
  }
  delay(5);
}


void loop() {
  int n = Serial1.available();
  if (n > 4) {
    int16_t v[2]; //-128..127
    Serial1.readBytes((char*)v, 4);
    setThrottle(v[0], v[1]);
  }
  delay(2);
}


//
