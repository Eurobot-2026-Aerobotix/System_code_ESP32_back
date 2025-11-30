#include <Arduino.h>
int motorPinsA[4] = {18, 12, 21, 22};
int motorPinsB[4] = {19, 14, 20, 33};

int encoderPinsA[4] = {35, 25, 26, 27};
int encoderPinsB[4] = {32, 34, 39, 36};

int minPWM[4] = {80, 80, 80, 80};

#define FAST_READ(pin) ((pin)<32 ? ((GPIO.in >> (pin)) & 0x1) : ((GPIO.in1.val >> ((pin)-32)) & 0x1))
#define SGN(a) ((a<0)?-1:1)

volatile long encoderCount[4] = {0, 0, 0, 0};

float kp[4] = {0.8, 0.8, 0.8, 0.8};
float ki[4] = {5, 5, 5, 5};
float kd[4] = {0.05, 0.05, 0.05, 0.05};

float integral[4] = {0, 0, 0, 0};
float last_error[4] = {0, 0, 0, 0};

long target_encoder_count[4] = {0, 0, 0, 0};

long downPosition[4] = {0, 0, 0, 0};
long midPosition[4] = {5000, 5000, 5000, 5000};
long upPosition[4] = {10000, 10000, 10000, 10000};

unsigned long last_time = 0;

void IRAM_ATTR encoderA_ISR_0(){ 
    bool A = FAST_READ(encoderPinsA[0]);
    bool B = FAST_READ(encoderPinsB[0]);
    encoderCount[0] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_0(){ 
    bool A = FAST_READ(encoderPinsA[0]);
    bool B = FAST_READ(encoderPinsB[0]);
    encoderCount[0] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_1(){ 
    bool A = FAST_READ(encoderPinsA[1]);
    bool B = FAST_READ(encoderPinsB[1]);
    encoderCount[1] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_1(){ 
    bool A = FAST_READ(encoderPinsA[1]);
    bool B = FAST_READ(encoderPinsB[1]);
    encoderCount[1] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_2(){ 
    bool A = FAST_READ(encoderPinsA[2]);
    bool B = FAST_READ(encoderPinsB[2]);
    encoderCount[2] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_2(){ 
    bool A = FAST_READ(encoderPinsA[2]);
    bool B = FAST_READ(encoderPinsB[2]);
    encoderCount[2] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_3(){ 
    bool A = FAST_READ(encoderPinsA[3]);
    bool B = FAST_READ(encoderPinsB[3]);
    encoderCount[3] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_3(){ 
    bool A = FAST_READ(encoderPinsA[3]);
    bool B = FAST_READ(encoderPinsB[3]);
    encoderCount[3] += (A != B) ? +1 : -1;
}


void send_values(float s, float ts, long pos, long tgt){
    Serial.write((uint8_t)0xAA); Serial.write((uint8_t*)&s, sizeof(s));
    Serial.write((uint8_t)0xAB); Serial.write((uint8_t*)&ts, sizeof(ts));
    Serial.write((uint8_t)0xAC); Serial.write((uint8_t*)&pos, sizeof(pos));
    Serial.write((uint8_t)0xAD); Serial.write((uint8_t*)&tgt, sizeof(tgt));
}

void setMotor(int speed, int motorIndex){  
  if(abs(speed)<minPWM[motorIndex]){
    speed = 0;
  }
  speed = constrain(speed , -255 , 255);
  if(speed > 0){
    analogWrite(motorPinsA[motorIndex], speed);
    analogWrite(motorPinsB[motorIndex], 0);
  }else{
    analogWrite(motorPinsA[motorIndex], 0);
    analogWrite(motorPinsB[motorIndex], -speed);
  }
}
void setup(){
    Serial.begin(115200);

    for(int i=0; i<4; i++){
        pinMode(encoderPinsA[i], INPUT_PULLUP);
        pinMode(encoderPinsB[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[0]), encoderA_ISR_0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[0]), encoderB_ISR_0, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[1]), encoderA_ISR_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[1]), encoderB_ISR_1, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[2]), encoderA_ISR_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[2]), encoderB_ISR_2, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[3]), encoderA_ISR_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[3]), encoderB_ISR_3, CHANGE);

    delay(2000);
    last_time = millis()-10;
}
void applyPositionPID(float dt, int motorIndex){
    float error = target_encoder_count[motorIndex] - encoderCount[motorIndex];
    integral[motorIndex] += error * dt;
    if(SGN(last_error[motorIndex]) != SGN(error)){
      integral[motorIndex] = 0;
    }
    integral[motorIndex] = constrain(integral[motorIndex], -500.0f, 500.0f);
    float derivative = (error - last_error[motorIndex]) / dt;
    float speedCmd = kp[motorIndex] * error + ki[motorIndex] * integral[motorIndex] + kd[motorIndex] * derivative;

    last_error[motorIndex] = error;
    setMotor((int)speedCmd, motorIndex);
}

void loop(){
    unsigned long currentTime = millis();
    float dt = (currentTime - last_time) / 1000.0;
    last_time = currentTime;
    for (int i=0; i<4; i++){
        applyPositionPID(dt, i);
    }
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command[0] == 'u' || command[0] == 'm' || command[0] == 'd'){
            for(int i=0; i<4; i++){
                if (command[i] == 'u'){
                    target_encoder_count[i] = upPosition[i];
                }else if (command[i] == 'm'){
                    target_encoder_count[i] = midPosition[i];
                }else if (command[i] == 'd'){
                    target_encoder_count[i] = downPosition[i];
                }
            }
        }else{
            target_encoder_count[(int)command[0]-48] = command.substring(1).toInt();
        }
    }
    delay(10);
}