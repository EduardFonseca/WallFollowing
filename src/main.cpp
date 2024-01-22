#include <Arduino.h>
#include <pico4drive.h>

// global variables
// uint8_t b;

#define pi 3.1415926535897932384626433832795
unsigned long interval, last_cycle;
unsigned long loop_micros;
byte encoderLpinALast, encoderRpinALast;

int durationL, durationR;//the number of the pulses
boolean DirectionL, DirectionR;//the rotation direction

int left_hand_rule = 0;
int flag_movement = 0;

int wheel_radius = 35; // milimiters
float wheel_base = 137.5;
float theta = 0.0;

float aux = 0;
float v, w;

// Posição inicial (x, y, theta):
double P0[3] = {0, 0, 0};
double dPdt[3] = {0,0,0};
double P[3] = {0, 0, 0};
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
float durationUltraL, distanceWallL;
unsigned long tesL = 0;
int flagPulseL = 0;

const int trigPinR = 27;
const int echoPinR = 26;
float durationUltraR, distanceWallR;
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
void wall_following_sm(fsm_t& fsm, float speed);

void EncoderInit();
void wheelSpeedLeft();
void wheelSpeedRight();

void UltraSonicInit();
void UltraSonicPulse();
void UltraDistL();
void UltraDistR();

void moveForward(float pwm, float PID_L, float PID_R);
void moveBackward(float pwm);
void turnLeft(float pwm);
void turnRight(float pwm);
void stop();

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


      // TODO: com o encoder de cada roda e possivel saber a distancia que cada uma andou e assim ver angulo e posicao do robo
      // calcular a distancia andada em linha reta antes de virar e dps de virar 

      float omega_L = (pi/180.0) * ((durationL * 9)/24)/(dt/1000);
      float omega_R = (pi/180.0) * (-1)*((durationR * 9)/24)/(dt/1000);

      float lin_vel_L = wheel_radius * omega_L / 1000;
      float lin_vel_R = wheel_radius * omega_R / 1000;
      

      if (Serial.available()) {  // Only do this if there is serial data to be read
        uint8_t b = Serial.read();       
        Serial.println(b);
        if (b == '-') aux -= 0.2;  // Press '-' to decrease the frequency
        if (b == '+') aux += 0.2; // Press '+' to increase the frequency
      }  

      // float dtheta = (omega_R - omega_L)*wheel_radius / wheel_base;

      // theta += dtheta;
      // PID_L = PIDControl_L(lin_vel_L, 0.22, dt); // TODO: ENTENDER COMO TUNAR O PID E COMO FAZER ELE ATUAR NO PWM 
      // if(theta > 3.1415/2 || theta < -3.1415/2){
      //   aux = 0;
      //   theta = 0;
      // }

      v = (lin_vel_L+lin_vel_R)/2;
      w = (lin_vel_R-lin_vel_L)/(wheel_base/1000);

            // Modelo diferencial do robô
      dPdt[0] = v * cos(P0[2]);    
      dPdt[1] = v * sin(P0[2]);
      dPdt[2] = w;
      
      P0[0] = P0[0] + dPdt[0] * dt/1000;
      P0[1] = P0[1] + dPdt[1] * dt/1000;
      P0[2] = P0[2] + dPdt[2] * dt/1000;

      if(P0[2]>= pi/2 || P0[2] <= -pi/2 || P0[0]<-0.30){
        aux = 0;
        P0[2] = 0;
      }

      while(P0[2] > 2*pi){
        P0[2] = P0[2] - 2*pi;
      }

      UltraSonicPulse();
      wall_following_sm(fsm_wall, aux);
      // moveForward(aux, PID_L, 0.0);
      // turnRight(aux);

      pico4drive_update();

      // Serial.print(" PulseL:");
      // Serial.print(durationL);
      // Serial.print(" PulseR:");
      // Serial.print(durationR);
      // Serial.print(" lin_vel_L:");
      // Serial.print(lin_vel_L);
      // Serial.print(" Lin_vel_R:");
      // Serial.print(lin_vel_R);
      Serial.print("dt:");
      Serial.print(dt);
      // Serial.print(" distanceWallR: ");
      // Serial.print(distanceWallR);
      // Serial.print(" distanceWallL:");
      // Serial.print(distanceWallL);
      // Serial.print(" aux:");
      // Serial.print(aux);      
      Serial.print(" state:");
      Serial.print(fsm_wall.state);
      Serial.print(" v:");
      Serial.print(v);
      Serial.print(" w:");
      Serial.print(w);
      Serial.print(" x:");
      Serial.print(P0[0]);
      Serial.print(" y:");
      Serial.print(P0[1]);
      Serial.print(" theta:");
      Serial.print(P0[2]);
      Serial.print(" teste:");
      Serial.println("");

      
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

void wall_following_sm(fsm_t& fsm, float speed){
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

  int wall_tresh = 10;
  if(fsm.state == 0 && on_button_state == 1){
    // TODO: IMPLEMENTAR O QUE INICIA A MAQUINA DE ESTADOS
    fsm.new_state = 1;
  }else if(fsm.state == 1 && distanceWallL > wall_tresh){
    // TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 2 SE HOUVER ABERTURA NA ESQUERDA
    // virar 90 graus para a esquerda e voltar para o estado 1
    P[0]=P0[0];
    P[1]=P0[1];
    P[2]=P0[2];
    fsm.new_state = 2;
  }else if(fsm.state == 1 && distanceWallL < wall_tresh && distanceWallR < wall_tresh){
    // TODO: IMPLEMENTAR MUDANCA DE ESTADO PARA O 3 SE HOUVER PAREDE NA FRENTE E ABERTURA NA DIREITA
    // virar para a direita ate que a frente do robo esteja livre e assim volte para o estado 1
    fsm.new_state = 3;
  }else if(flag_movement == 1){
    // retorno para o estado 1
    fsm.new_state = 1;
  }
  // set new state
  set_state(fsm, fsm.new_state);

  if(fsm.state == 0){
    // STANDBY
    stop();
    flag_movement = 0;
  }else if(fsm.state == 1){
    // GO FORWARD
    moveForward(speed,0,0);    
    flag_movement = 0;
  }else if(fsm.state == 2){
    // TURN LEFT
    if((abs(P[0]-P0[0]) < 0.1) && (abs(P[1]-P0[1]) < 0.1)){
      moveForward(speed,0,0);
    }else{
      if(abs(P[2]-P0[2]) < pi/2){
        turnLeft(speed);
      }else{
        fsm.new_state = 1;
      }
    }
  }else if(fsm.state == 3){
    // TURN RIGHT
    turnRight(speed);
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
    distanceWallL = 0.5*(344*durationUltraL)/10000.0; // return in CM
    flagPulseL = 0;
    tesL = 0;
  }
}

void UltraDistR (){
  if(tesR == 0 ){
    tesR = micros();
  }else{
    durationUltraR = micros() - tesR;
    distanceWallR = 0.5*(344*durationUltraR)/10000.0; // return in CM
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

void turnLeft(float pwm){
  // Implementar o controle
  pico4drive_set_motor_pwm(DRV_1, pwm);
  pico4drive_set_motor_pwm(DRV_4, pwm); 
}

void turnRight(float pwm){
  // Implementar o controle
  pico4drive_set_motor_pwm(DRV_1, -pwm);
  pico4drive_set_motor_pwm(DRV_4, -pwm); 
}

void stop(){
  // Implementar o controle
  pico4drive_set_motor_pwm(DRV_1, 0);
  pico4drive_set_motor_pwm(DRV_4, 0);
}