#include <Arduino.h>
#include <pico4drive.h>

// global variables
// uint8_t b;
unsigned long interval, last_cycle;
unsigned long loop_micros;
byte encoderLpinALast, encoderRpinALast;

int durationL, durationR;//the number of the pulses
boolean DirectionL, DirectionR;//the rotation direction

int left_hand_rule = 0;
int flag_movement = 0;

int wheel_radius = 35; // milimiters

//function global var
// int battery_mV;
// int on_button_state;
// uint32_t on_button_press_count;

const byte encoderLpinA = 0;//A pin -> the interrupt pin 0
const byte encoderLpinB = 1;//B pin -> the digital pin 3

const byte encoderRpinA = 2;//A pin -> the interrupt pin 0
const byte encoderRpinB = 3;//B pin -> the digital pin 3

// ultrasonic pins
const int trigPinL = 8;
const int echoPinL = 9;
float durationUltraL, distanceL;
unsigned long tesL = 0;
int flagPulseL = 0;

const int trigPinR = 27;
const int echoPinR = 26;
float durationUltraR, distanceR;
unsigned long tesR = 0;
int flagPulseR = 0;

// structure declaration
typedef struct{
  int state, new_state, prev_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tis, tes;
} fsm_t;


// initializing the PID VAR for Left wheel
float PID_L;
float error_L, last_error_L;
float Kp_L, Ki_L, Kd_L;
float integral_L, derivative_L, proportional_L;
float max_lin_vel_L = 100; //TODO: VERIFICAR QUAL E A VELOCIDADE PAR O PWM 1 para em seguida fazer a conversao da velocidade para um pwm

// initializing the PID VAR for Left wheel
float error_R, last_error_R;
float Kp_R, Ki_R, Kd_R;
float integral_R, derivative_R, proportional_R;
float max_lin_vel_R = 100; //TODO: VERIFICAR QUAL E A VELOCIDADE PAR O PWM 1 para em seguida fazer a conversao da velocidade para um pwm

// Funtions declaration
void set_state(fsm_t& fsm, int new_state);
void wall_following_sm(fsm_t& fsm, int mode);

void EncoderInit();
void wheelSpeedLeft();
void wheelSpeedRight();

void UltraSonicInit();
void UltraSonicPulse();
void UltraDistL();
void UltraDistR();

void moveForward(float pwm, float PID_L, float PID_R);
void moveBackward(float pwm);

float PIDControl_L(float pulse, float setpoint, float dt);
float PIDControl_R(float pulse, float setpoint, float dt);



fsm_t fsm_wall;



void setup() {
  // put your setup code here, to run once:
  pico4drive_init();
  interval = 40;
  last_cycle = millis();

  set_state(fsm_wall, 0);

  EncoderInit();
  UltraSonicInit();

  // start the gains
  Kp_L = 1;
  Ki_L = 0.0;
  Kd_L = 0.0;
}

void loop() {
  // modify variable value through serial port
  // if (Serial.available()) {  // Only do this if there is serial data to be read
  //     b = Serial.read();       
  // }

  // Do this only every "interval" miliseconds 

  unsigned long now = millis();
    if (now - last_cycle > interval - 1) {
      float dt = now - last_cycle;
      loop_micros = micros();
      last_cycle = now;

      // dt =  dt/1000000;
      // moveForward(0.3, PID_L, 0);
      float omega_L = (3.14/180.0) * ((durationL * 9)/24)/(dt/1000);
      float omega_R = (3.14/180.0) * ((durationR * 9)/24)/(dt/1000);
      // float omega_R = durationR * 1000000 / dif;

      float lin_vel_L = wheel_radius * omega_L / 1000;
      float lin_vel_R = wheel_radius * omega_R / 1000;

      // PID_L = PIDControl_L(lin_vel_L, 0.22, dt); // TODO: ENTENDER COMO TUNAR O PID E COMO FAZER ELE ATUAR NO PWM 

      // if (Serial.available()) {  // Only do this if there is serial data to be read
      //   uint8_t b = Serial.read();       
      //   if (b == '-') Kp_L = Kp_L - 0.1;  // Press '-' to decrease the frequency
      //   if (b == '+') Kp_L = Kp_L + 0.1; // Press '+' to increase the frequency
      // }  

      UltraSonicPulse();
  

      pico4drive_update();
      Serial.print("Bat_V: ");
      Serial.print(battery_mV);

      Serial.print(" PulseL:");
      Serial.print(durationL);
      Serial.print(" PulseR:");
      Serial.print(durationR);
      Serial.print(" lin_vel_L:");
      Serial.print(lin_vel_L);
      Serial.print(" Lin_vel_R:");
      Serial.print(lin_vel_R);
      Serial.print(" dt: ");
      Serial.print(dt);
      Serial.print(" distanceR: ");
      Serial.print(distanceR);
      Serial.print(" distanceL:");
      Serial.println(distanceL);



      
      durationL = 0;
      durationR = 0;
  }
}

// put function definitions here:
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.prev_state = fsm.state;
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

void wall_following_sm(fsm_t& fsm,  int mode){
  /*
  Wall following state machine
  args:
    mode: 0 - wall following left
          1 - wall following right
  states:
    0 - standby
    1 - go forward
    2 - turn left
    3 - turn right
    4 - turn around
    5 - stop
  */
  if(mode == 0){
    if(fsm.state == 0 /* && flag_movement == 1*/){
      // TODO: IMPLEMENTAR O QUE INICIA A MAQUINA DE ESTADOS
      fsm.new_state = 1;
    }else if(fsm.state == 1){
      // TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 2 SE HOUVER ABERTURA NA ESQUERDA
      fsm.new_state = 2;
    }else if(fsm.state == 1){
      // TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 3 SE HOUVER PAREDE NA FRENTE E ABERTURA NA DIREITA
      fsm.new_state = 3;
    }else if(fsm.state == 1){
      // TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 4 SE HOUVER NAO HOUVER ABERTURA E NAO FOR O ENDPOINT
      fsm.new_state = 4;
    }else if((fsm.state == 2 || fsm.state == 3 || fsm.state == 4) && flag_movement == 1){
      // TODO: Implementar a mudanca de estado para 1
      fsm.new_state = 1;
    }
    //else if(endpoind == 1){
    //  TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 5
    //  fsm.new_state = 5;
    //}

    // set new state
    set_state(fsm, fsm.new_state);

    if(fsm.state == 0){
      // STANDBY
    }else if(fsm.state == 1){
      // GO FORWARD
    }else if(fsm.state == 2){
      // TURN LEFT
    }else if(fsm.state == 3){
      // TURN RIGHT
    }else if(fsm.state == 4){
      // TURN AROUND
    }else if(fsm.state == 5){
      // STOP
    }
  }
}

void EncoderInit()
{
  DirectionL = true;//default -> Forward
  DirectionR = true;//default -> Forward
  pinMode(encoderLpinA,INPUT);
  pinMode(encoderLpinB,INPUT);
  pinMode(encoderRpinB,INPUT);
  pinMode(encoderRpinA,INPUT);
  attachInterrupt(encoderLpinA, wheelSpeedLeft, RISING);
  attachInterrupt(encoderRpinA, wheelSpeedRight, RISING);
}

// 9 graus de rotacao na roda ==> 24 pulsos

void wheelSpeedLeft()
{
  int Lstate = digitalRead(encoderLpinA);
  if((encoderLpinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderLpinB);
    if(val == LOW && DirectionL)
    {
      DirectionL = false; //Reverse
    }
    else if(val == HIGH && !DirectionL)
    {
      DirectionL = true;  //Forward
    }
  }
  encoderLpinALast = Lstate;

  if(!DirectionL)  durationL++;
  else  durationL--;
}

void wheelSpeedRight()
{
  int LstateR = digitalRead(encoderRpinA);
  if((encoderRpinALast == LOW) && LstateR==HIGH)
  {
    int val = digitalRead(encoderRpinB);
    if(val == LOW && DirectionR)
    {
      DirectionR = false; //Reverse
    }
    else if(val == HIGH && !DirectionR)
    {
      DirectionR = true;  //Forward
    }
  }
  encoderRpinALast = LstateR;

  if(!DirectionR)  durationR++;
  else  durationR--;
}

void UltraSonicInit(){
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  attachInterrupt(echoPinL, UltraDistL , CHANGE);
  attachInterrupt(echoPinR, UltraDistR , CHANGE);
}

void UltraDistL (){
  if(tesL == 0 ){
    tesL = micros();
  }else{
    durationUltraL = micros() - tesL;
    distanceL = 0.5*(344*durationUltraL)/10000.0; // return in CM
    flagPulseL = 0;
    tesL = 0;
  }
}

void UltraDistR (){
  if(tesR == 0 ){
    tesR = micros();
  }else{
    durationUltraR = micros() - tesR;
    distanceR = 0.5*(344*durationUltraR)/10000.0; // return in CM
    flagPulseR = 0;
    tesR = 0;
  }
}

void UltraSonicPulse(){
  // PENSAR EM UMA FORMA DE implementar isso assincronamente 
  if(flagPulseL == 0){
    digitalWrite(trigPinL,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinL,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL,LOW);
    flagPulseL = 1;
  }
  if(flagPulseR == 0){
    digitalWrite(trigPinR,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR,LOW);
    flagPulseR = 1;
  }


  // durationUltraL = pulseIn(echoPinL,HIGH,50000);
  // durationUltraR = pulseIn(echoPinR,HIGH,50000);

  // distanceL = 0.5*(344*durationUltraL)/10000.0; // return in CM
  // distanceR = 0.5*(344*durationUltraR)/10000.0;
  // durationUltraL = pulseIn(echoPinL, HIGH);
}



float PIDControl_L(float lin_vel, float setpoint, float dt){
  // Implementar o controle
  // convert linear velocity to pulses


  last_error_L = error_L;
  error_L = setpoint - lin_vel;
  derivative_L = (error_L - last_error_L) / dt;
  integral_L = integral_L + error_L * dt;
  proportional_L = error_L;

  return proportional_L * Kp_L + integral_L * Ki_L + derivative_L * Kd_L;
}

float PIDControl_R(float lin_vel, float setpoint, float dt){
  // Implementar o controle
  last_error_R = error_R;
  error_R = setpoint - lin_vel;
  derivative_R = (error_R - last_error_R) / dt;
  integral_R = integral_R + error_R * dt;
  proportional_R = error_R;

  return proportional_R * Kp_R + integral_R * Ki_R + derivative_R * Kd_R;
}

void moveForward(float pwm, float PID_L, float PID_R){
  // Implementar o controle
  pico4drive_set_motor_pwm(DRV_1, pwm);
  pico4drive_set_motor_pwm(DRV_4, -pwm); 
}
void moveBackward(float pwm){
  // Implementar o controle
  pico4drive_set_motor_pwm(DRV_1, -pwm);
  pico4drive_set_motor_pwm(DRV_4, pwm); 
}

// void turnLeft(){
//   // Implementar o controle
//   pico4drive_set_motor_pwm(DRV_1, 0);
//   pico4drive_set_motor_pwm(DRV_4, 0); 
// }

// void turnRight(){
//   // Implementar o controle
//   pico4drive_set_motor_pwm(DRV_1, 0);
//   pico4drive_set_motor_pwm(DRV_4, 0); 
// }

// void stop(){
//   // Implementar o controle
//   pico4drive_set_motor_pwm(DRV_1, 0);
//   pico4drive_set_motor_pwm(DRV_4, 0);
// }