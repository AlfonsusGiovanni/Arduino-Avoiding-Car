#include "Servo.h"
#include "NewPing.h"

/*KONFIGURASI PIN MOTOR*/
#define AIN1  3
#define AIN2  2
#define STDBY 4
#define BIN1  5
#define BIN2  10
#define PWMA  11
#define PWMB  6

/*KONFIGURASI PIN SERVO*/
#define servo_pin 9
Servo myservo;

/*KONFIGURASI PIN ULTRASONIC*/
#define trigger A0
#define echo A1
#define max_distance 200
#define threshold 15
NewPing sonar(trigger, echo, max_distance);

/*KONFIGURASI PIN LDR*/
#define LDR A4

/*KONFIGURASI PIN LED*/
#define LED 12

/*---START VARIABLE---*/
int
df, dl, dr, fv, counter,
delay_time = 450, ldr_val;

bool
stateF = false,
stateL = false,
stateR = false;

char state;
/*---END VARIABLE---*/

void setup() 
{
  Serial.begin(9600);
  myservo.attach(servo_pin);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STDBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(LED, OUTPUT);

  pinMode(LDR, INPUT);

  check_ping();
}

void loop() 
{
  
  myservo.write(90);
  fv = sonar.ping_cm();

  ldr_val = analogRead(LDR);

  Serial.println(ldr_val);

  if(ldr_val < 100)
  {
    digitalWrite(LED, HIGH);
  }

  else if(ldr_val > 100)
  {
    digitalWrite(LED, LOW);
  }
  
  if(fv > 0 && fv <= threshold)
  {
    run_motor('S');
    digitalWrite(LED, HIGH);
    delay(500);
    check_ping();
    check_logic();
  }

  else
  {
    state = 'F';
  }

  if(state == 'F')
  {
    run_motor('F');
    digitalWrite(LED, HIGH);
  }

  if(state == 'L')
  {
    run_motor('L');
    delay(delay_time);
    state = 'F';
  }

  else if(state == 'R')
  {
    run_motor('R');
    delay(delay_time);
    state = 'F';
  }

  else if(state == 'T')
  {
    if(counter == 0)
    {
      run_motor('R');
      delay(delay_time);
      counter = 1;
      state = 'F';
    }

    else if(counter == 1)
    {
      run_motor('R');
      delay(delay_time);
      counter = 0;
      state = 'F';
    }
  }
}

void check_ping() {
  myservo.write(90);
  df = sonar.ping_cm();

  //Serial.print("Front Value: ");
  //Serial.println(df);

  delay(500);

  myservo.write(0);
  dr = sonar.ping_cm();

  //Serial.print("Right Value: ");
  //Serial.println(dr);

  delay(500);

  myservo.write(180);
  dl = sonar.ping_cm();

  //Serial.print("Left Value: ");
  //Serial.println(dl);

  delay(500);

  myservo.write(90);
}

void check_logic() {
  if(df <= threshold && dl <= threshold && dr <= threshold)
  {
    state = 'T';
  }

  else if (df <= threshold && dl <= threshold && dr > threshold) 
  {
    state = 'R';
  }

  else if (df <= threshold && dl > threshold && dr <= threshold) 
  {
    state = 'L';
  }

  else 
  {
    state = 'F';
  }
}

void run_motor(char mode) {
  switch (mode) {
    case 'F':
      digitalWrite(STDBY, HIGH);

      analogWrite(PWMA, 125);
      analogWrite(PWMB, 125);

      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      break;

    case 'B':
      digitalWrite(STDBY, HIGH);

      analogWrite(PWMA, 125);
      analogWrite(PWMB, 125);

      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      break;

    case 'L':
      digitalWrite(STDBY, HIGH);

      analogWrite(PWMA, 125);
      analogWrite(PWMB, 125);

      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      break;

    case 'R':
      digitalWrite(STDBY, HIGH);

      analogWrite(PWMA, 125);
      analogWrite(PWMB, 125);

      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      break;

    case 'S':
      digitalWrite(STDBY, HIGH);

      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      break;
  }
}
