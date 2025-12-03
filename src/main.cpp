#include <Arduino.h>
#include <ESP32Servo.h>


#define FAST_READ(pin) ((pin)<32 ? ((GPIO.in >> (pin)) & 0x1) : ((GPIO.in1.val >> ((pin)-32)) & 0x1))
#define SGN(a) ((a<0)?-1:1)

int motorPinsA[4] = {19, 14, 23, 16};
int motorPinsB[4] = {2, 4, 5, 15};
int motorPinsEnable[4] = {18, 12, 21, 22};

int encoderPinsA[4] = {35, 25, 26, 27};
int encoderPinsB[4] = {32, 34, 39, 36};

const int middle_servo_pin = 33;
const int left_servo_pin = 13;
const int right_servo_pin = 17;

Servo middle_servo;
Servo left_servo;
Servo right_servo;
int middle_servo_pos_0 = 1000;
int middle_servo_pos_1 = 2000;

int left_servo_pos_0 = 1000;
int left_servo_pos_1 = 2000;

int right_servo_pos_0 = 1000;
int right_servo_pos_1 = 2000;

int minPWM[4] = {80, 80, 80, 80};

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

void setupServos(){
    middle_servo.setPeriodHertz(50);
    middle_servo.attach(middle_servo_pin, 500, 2500);
    middle_servo.write(middle_servo_pos_0);
    left_servo.setPeriodHertz(50);
    left_servo.attach(left_servo_pin, 500, 2500);
    left_servo.write(left_servo_pos_0);
    right_servo.setPeriodHertz(50);
    right_servo.attach(right_servo_pin, 500, 2500);
    right_servo.write(right_servo_pos_0);
}

void setMotor(int pwm, int motorIndex){  
    if(abs(pwm)<minPWM[motorIndex]){
    pwm = 0;
    }
    pwm = constrain(pwm , -255 , 255);
    digitalWrite(motorPinsEnable[motorIndex], abs(pwm));
    if (pwm > 0){
        digitalWrite(motorPinsA[motorIndex], 1);
        digitalWrite(motorPinsB[motorIndex], 0);
    }
    else if (pwm < 0){
        digitalWrite(motorPinsA[motorIndex], 0);
        digitalWrite(motorPinsB[motorIndex], 1);
    }else{
        digitalWrite(motorPinsEnable[motorIndex], 255);
        digitalWrite(motorPinsA[motorIndex], 1);
        digitalWrite(motorPinsB[motorIndex], 1);
    }
}
void setup(){
    Serial.begin(115200);

    for(int i=0; i<4; i++){
        pinMode(encoderPinsA[i], INPUT_PULLUP);
        pinMode(encoderPinsB[i], INPUT_PULLUP);
        pinMode(motorPinsA[i], OUTPUT);
        pinMode(motorPinsB[i], OUTPUT);
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
    setupServos();
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

void ba33ed_3la_ba3dhou(){
    middle_servo.write(middle_servo_pos_1);
}

void lemm_l_ba3dhou(){
    middle_servo.write(middle_servo_pos_0);
}

void estwi_l_crates(){
    left_servo.write(left_servo_pos_1);
    right_servo.write(right_servo_pos_1);
}

void khabbi_l_kabecha(){
    left_servo.write(left_servo_pos_0);
    right_servo.write(right_servo_pos_0);
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
    ba33ed_3la_ba3dhou();
    estwi_l_crates();
    delay(1000);
}