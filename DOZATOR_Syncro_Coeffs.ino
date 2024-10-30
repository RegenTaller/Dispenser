#include <avr/eeprom.h>

#define _sign(x) ((x) > 0 ? 1 : -1)  //Сигнум для смены приращения скорости на уменьшение


#define SPEED_1 8  //8 исходно //6 при одном таймере
#define IN_1 9
#define IN_2 10

#define SPEED_2 11  //13 исходно// 11 после приведения к одной частоте //7 при одном таймере
#define IN_3 12
#define IN_4 13  //исх 11

#define ENC_1 3  //энкодер первый таймер 0
#define ENC_12 2

#define ENC_2 20  //энкодер второй таймер 1
#define ENC_22 21

const bool level = 1;  //направление для всех пинов

int Dduty = 0;


int _accel = 18000;    //ускорение в отсчётах энкодера в секунду
int _maxSpeed = 8000;  //максимальная скорость в отсчётах энкодера в секунду

const uint8_t _dt = 8;          //Временной шаг вызова функции регулирования
float _dts = (float)_dt / 1000;  //Временной шаг для подсчёта в единицах в секунду теор. значений скоростей V и позиций pos
uint32_t _tmr2 = 0;              //Переменная таймера comp_cur_pos для подсчёта значений V и pos
long _targetPos = 0;             //Целевое положение в отсчётах энкодера
int dir = -1;                    //Направление 1 - вперёд // -1 - назад

long controlPos = 0;     //Теоретическое положение, идущее в PID
float controlSpeed = 0;  //Теоретическая скорость на шаге

int _minDuty = 0, _maxDuty = 255;  //Минимальное значение страгивания ДПТ: (_minDuty-255)
float _k = 1.0;

///////////////////////////////////////////////////////////
//////////ПЕРЕМЕННЫЕ ЭНКОДЕРОВ И РЕАЛЬНОЙ СКОРОСТИ/////////

volatile int lastEncoded1 = 0;    // Here updated value of encoder store.
volatile long encoderValue1 = 0;  // Raw encoder value
volatile long encREF1 = 0;        //сбрасываемое значениеэнкодера для скорости

volatile int lastEncoded2 = 0;    // Here updated value of encoder store.
volatile long encoderValue2 = 0;  // Raw encoder value
volatile long encREF2 = 0;        //сбрасываемое значениеэнкодера для скорости

int16_t encREF1copy = 0;
int16_t encREF2copy = 0;
float Velocity1 = 0.0;  //Переменная мгновенной скорости 1
float Velocity2 = 0.0;  //Переменная мгновенной скорости 2
//uint8_t freq1 = 10; // частота опроса

uint8_t per1 = 50;  ///freq1; //период опроса скоростей 1000/60 = 17 мс
uint32_t t1 = 0;    //начальный момент времени unsigned long
uint32_t t2 = 0;    //unsigned long 4

int _buf[3];
byte _count = 0;
float _middle_f = 0;



float T_Theoretic = 0;

int ratio = 8344;  //8433 = 28*298
///////////////////////////////////////////////////////////////////////////
////////////////////////////////ПИД РЕГУЛЯТОР//////////////////////////////

long _previnput = 0, _previnput2 = 0;
float integral = 0.0000;
float integral2 = 0.0000;

bool cutoff = 0;
int stopzone = 10, stopzone2 = 10;


float kp = 0.35;  // 2x: 0.35;  4x: 0.085		// (знач. по умолчанию)0.1 0.5 0.05
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
float ki = 0.00000095;  //2x: 0.000005 4x: 0.00000011 0.00000015 для высок.
// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
float kd = 1;  //2x: 35 - 2 прерывания 4x: 0.75 7.75 для высок скоростей
//////////////////////////////////////////////////////////////////////

////////////////Альтернативная "АНАЛИТИЧЕСКАЯ" кривая/////////////////
float ta = 0;  //конечный момент времени
float tb = 0;  //время окончания разгона
float tc = 0;  //время окончания равномерного движения

float xb = 0;
float xc = 0;
float curTime = 0;
long controlPos2 = 0;
int N = 0;  // Число шагов дискретизации

int8_t curve = 1;  //Тип кривой. -1 - пересечение парабол 2 - нормальный случай, 1 - только ускорение и равномерное

long timer = 0;  //ControlPos2 таймер
long f = 0;      //счётчик приращения

/////////////////////////////////////////////////////////////////////

//////////////////////////////Сбор данных////////////////////////////
uint8_t marker = 0;  //маркер прекращения
#define arr_sizePos1 40
#define arr_sizePos2 4
long EncStat1[arr_sizePos1][arr_sizePos2];
long EncStat2[arr_sizePos1][arr_sizePos2];
unsigned long del = 0;  //Обнуление таймера микрос

long tmrexp = 0;  //таймер цикла сбора

uint16_t cnn = 0, cnn1 = 0, cnn2 = 0;  //Счётчики вызовов

int16_t du = 0, du1 = 0;
/////////////////////////////////////////////////////////////////////

uint8_t n = 0;
int index_cur = 0;
// inline __attribute__((always_inline))

// void test(void) {

//   asm ("nop");

//   comp_cur_pos();
//   int PWM1 = PIDcalc(4000, encoderValue1, kp, ki, kd, _dts, dir, cutoff);
//   int PWM2 = PIDcalc2(3500, encoderValue2, kp, ki, kd, _dts, dir, cutoff);

//   movement(PWM1, 1);
//   movement(PWM2, 2);


  //Serial.println("wef");
  //encoded ();

     //store this value for next time

  //int8_t c = PIDcontrol(550, 350, true);
  //
  //Serial.println(_dutyF);
  //Serial.println();

  //PIDcalc(_targetPos, cur, kp, ki, kd, _dts);
  //setPins(1, 0, 150, 1);
  //d++;
  //PIDcalc(_targetPos, cur, kp, ki, kd, _dts);
  //setSpeed(Duty, 2);
  //MotMove(Duty, IN_1, IN_2, SPEED_1);

  //bitRead(PIND, ENC_22);


  // uint8_t timer = digitalPinToTimer(ENC_22);
  // //Serial.println(digitalPinToTimer(2));
	// uint8_t bit = digitalPinToBitMask(ENC_22);
	// uint8_t port = digitalPinToPort(ENC_22);
	// //if (port == NOT_A_PIN) return LOW;
	// // If the 2 that support PWM output, we need to turn it off
	// // before getting a digital reading.
	// if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	// if (*portInputRegister(port) & bit) return HIGH;
	// return LOW;




//   //setSpeed(PIDcontrol(_targetPos, cur, true), 2);
//   //setSpeed(100, 2);
//   //PIDcontrol(550, 350, true);


//   //comp_cur_pos();

//   //digitalRead(2);
// }


// =========================================


volatile uint16_t cnt_ovf = 0;
// void setup() {
//   Serial.begin(2000000);
//   Calculus(_accel, _maxSpeed, _targetPos, _dts);
//   TCCR1A = TCCR1B = TCNT1 = cnt_ovf = 0;  // Сброс таймера
//   TIFR1 = (1 << TOV1);
//   TIMSK1 = (1 << TOIE0);                  // Прерывание переполнения
//   TCCR1B = (1 << CS10);                   // Старт таймера
//   test();                                 // тест
//   TCCR1B = 0;                             // остановить таймер
//   uint32_t count = TCNT1 - 2;             // минус два такта на действия
//   count += ((uint32_t)cnt_ovf * 0xFFFF);  // с учетом переполнений

//   Serial.print("ticks: ");
//   Serial.println(count);
//   Serial.print("time (us): ");
//   Serial.println(count * (float)(1000000.0f / F_CPU), 4);
//   analogWrite(SPEED_1, 0);
//   analogWrite(SPEED_2, 0);
// }
// ISR(TIMER1_OVF_vect) {
//   cnt_ovf++;
// }

// void loop() {}
int pwm = 0;
//long err = 0;
long tpid = 0;

int PWM1 = 0, PWM2 = 0;

int flag = 0;
long flagtimer = 0;

bool offtrig = 0;

void setup() {

  // TCCR4B = (TCCR4B & 0xF8) |3; //Таймер 4 16 бит пин 11/8
  // TCCR1B = (TCCR1B & 0xF8) |3; //Таймер 1 16 бит пин 11/8

  Serial.begin(1000000);

  for (int i = 0; i < 10; i++) {

    pinMode(i, OUTPUT);
  }
  //pinMode(SPEED_1, HIGH);

  pinMode(ENC_1, INPUT_PULLUP);
  pinMode(ENC_12, INPUT_PULLUP);

  pinMode(4, INPUT_PULLUP);//Новый пин энкодера Старый:3
   digitalWrite(4, HIGH);

  digitalWrite(ENC_1, HIGH);   //turn pullup resistor on
  digitalWrite(ENC_12, HIGH);  //turn pullup resistor on

  pinMode(ENC_2, INPUT);
  pinMode(ENC_22, INPUT);

  digitalWrite(ENC_2, HIGH);   //turn pullup resistor on
  digitalWrite(ENC_22, HIGH);  //turn pullup resistor on

 

  //attachInterrupt(digitalPinToInterrupt(ENC_1), pin_A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_12), pin_B_ISR, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENC_1), updateEncoder1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC_12), updateEncoder1, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENC_2), updateEncoder2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC_22), updateEncoder2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_2), pin_A_ISR2, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENC_22), pin_B_ISR2, RISING);

  //setDeg(360); //Выставление значения цели в градусах
  //setTarget(1500);

  setRatio(8344, 2);
  setObor(1);
  //setMillimeters(0);
  //setSpeedMMS(2);
  setMinDuty(25);


  teor(_targetPos, _accel, _maxSpeed);
  Serial.println("DTS: " + String(_dts));
  Calculus(_accel, _maxSpeed, _targetPos, _dts);

  delay(1000);
  encoderValue1 = encoderValue2 = 0;

  //Serial.print(_targetPos*1.01);
  //Serial.print(_maxSpeed * 1.01 / 1000);
  //Serial.print(_duty);
  //Serial.print(-2);

  // Serial.println(_minDuty);

  Serial.println(((float)_maxSpeed * 1.01) / 1000);
  Serial.println((float)_targetPos * 1.01);



  t1 = t2 = flagtimer = millis();  //t1 used in velocities
  timer = millis();                //Таймер ControlPos2
  tmrexp = millis();               //Таймер прогоночного цикла
  //timer2 = millis();
  _tmr2 = millis();

  del = micros();  //Таймер пакетного вывода

  tpid = millis();



  // put your setup code here, to run once:
}

void loop() {

  // if (n == 0) {  //Стартовый прогон

  //   //digitalWrite(IN_1, 1);
  //   //digitalWrite(IN_2, 0);
  //   //analogWrite(SPEED_1, 70);
  //   //delay(250);
  //   n = 1;
  //   //analogWrite(SPEED_1, 0);
  //   Serial.println("Started");
  // }

  comp_cur_pos();  //cnn2


  // int intVariable = Serial.parseInt();
  // setObor(intVariable);

  //comp_cur_pos();
  if (millis() - tpid >= _dt) {

    PWM1 = PIDcalc(controlPos, encoderValue1, kp, ki, kd, _dt, dir, 0, offtrig);
    movement2(PWM1, 1);
    PWM2 = PIDcalc2(controlPos, encoderValue2, kp, ki, kd, _dt, dir, 0, offtrig);
    movement2(PWM2, 2);

    tpid = millis();
  }

  //int PWM2 = PIDcalc2(controlPos, encoderValue2, kp, ki, kd, _dts, dir, cutoff);

  // digitalWrite(IN_1, 1);
  // digitalWrite(IN_2, 0);
  // analogWrite(SPEED_1, 0);
  //movement(PWM1, 1);

  if (millis() - t2 >= 200) {

    //Velocities(30);
    //PWMPORT();
    POSITIONS();

    // if (_targetPos > encoderValue1 + stopzone && flag != 1 && flag < 2) {

      

    // }else if (flag < 2) {

    //   flag++;
    //   Serial.println("END Time: " + String(millis()-flagtimer));


    // }
    //VELS();
 
    t2 = millis();
    //Serial.println("in: "  + String(integral2));
  }



  //update_bufPos_short(EncStat1, del, controlPos, encoderValue1, PWM1);
  //update_bufPos_short(EncStat2, del, controlPos, encoderValue2, PWM2);
  //   index_cur++;

  //   if (index_cur == arr_sizePos1 ) {

  //     index_cur = 0;
  //     //print_bufPos_short(EncStat1);
  //     //print_bufPos_short(EncStat2);

  //   }
  //   // if (millis()-t2 >= 500) {


  //   //      Serial.println(encoderValue1);
  //   //      Serial.println(pwm);
  //   //      Serial.println(err);
  //   //      t2 = millis();
  //   // }

  //   marker++;

  //   if ((marker == arr_sizePos1 - 5)) {

  //     marker = 0;

  //     //print_bufPos(EncStat);;
  //   }

  //   //setSpeed2(PIDcalc(controlPos, encoderValue2, kp, ki, kd, _dts), 2);
  //   //setSpeed2(PIDcontrol2(controlPos, encoderValue2, true), 2);

  // } else if (n < 2) {

  //   n++;
  //   Serial.println("PIDcontrol cnn = " + String(cnn));
  //   Serial.println("PIDcalc cnn1 = " + String(cnn1));
  //   Serial.println("comp_cur_Pos cnn2 = " + String(cnn2));
  //   Serial.println(du);
  //   Serial.println(encoderValue1);
  //   Serial.println(controlPos);
  //   //Serial.println(controlPos);
  //   //Serial.println(encoderValue2);
  // }

  //Velocities(per1);
  //}
}


////////////////////"ЭНКОДЕРЫ"////////////////////////
void pin_A_ISR() {
  if (!(PINE & 0b00010000) == !(PING & 0b00100000)) { //2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000
    encoderValue1++;
    encREF1++;
  } else {
    encoderValue1--;
    encREF1--;
  }
}

void pin_B_ISR() {
  if (!(PINE & 0b00010000) == !(PING & 0b00100000)) {// 2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000
    encoderValue1--;
    encREF1--;
  } else {
    encoderValue1++;
    encREF1++;
  }
}

void pin_A_ISR2() {
  if (!(PIND & 0b00000010) == !(PIND & 0b00000001)) {  // 20 && 21 pins PD0 PD1    !(PIND & 0b00001000) == !(PIND & 0b00000100
    encoderValue2++;
    encREF2++;
  } else {
    encoderValue2--;
    encREF2--;
  }
}

void pin_B_ISR2() {
  if (!(PIND & 0b00000010) == !(PIND & 0b00000001)) {
    encoderValue2--;
    encREF2--;
  } else {
    encoderValue2++;
    encREF2++;
  }
}

void updateEncoder1() {

  int MSB1 = digitalRead(ENC_1);   //MSB = most significant bit
  int LSB1 = digitalRead(ENC_12);  //LSB = least significant bit


  int encoded1 = (MSB1 << 1) | LSB1;          //converting the 2 pin value to single number
  int sum1 = (lastEncoded1 << 2) | encoded1;  //adding it to the previous encoded1 value


  if (sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) {

    encoderValue1--;
    encREF1--;
    //Serial.println("Counter ClockWise");
  }

  if (sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) {

    encoderValue1++;
    encREF1++;
    //Serial.println("ClockWise");
  }

  lastEncoded1 = encoded1;  //store this value for next time
}

void updateEncoder2() {

  int MSB2 = digitalRead(ENC_2);   //MSB = most significant bit
  int LSB2 = digitalRead(ENC_22);  //LSB = least significant bit

  int encoded2 = (MSB2 << 1) | LSB2;          //converting the 2 pin value to single number
  int sum2 = (lastEncoded2 << 2) | encoded2;  //adding it to the previous encoded1 value

  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) {

    encoderValue2--;
    encREF2--;
    //Serial.println("Counter ClockWise");
  }

  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) {

    encoderValue2++;
    encREF2++;
    //Serial.println("ClockWise");
  }

  lastEncoded2 = encoded2;  //store this value for next time
}

///////////////////////////////////////////////////////////////////////////
//////////////////////////ГЛАВНЫЕ УПРАВЛЯЮЩИЕ//////////////////////////////

void comp_cur_pos() {  // USes timpos & _tmr2  (long encoder, uint8_t motor)

  unsigned long timpos = millis();

  if (timpos - _tmr2 >= _dt) {
    _dts = (timpos - _tmr2) / 1000.0;
    _tmr2 = millis();
    long err = _targetPos - controlPos;  // "ошибка" позиции
    if (err != 0) {
      if (_accel != 0) {
        bool thisDir = (controlSpeed * controlSpeed / _accel / 2.0 >= abs(err));  // пора тормозить (false до приближения к параболе торможения обр. от ускор.)
        controlSpeed += _accel * _dts * (thisDir ? -_sign(controlSpeed) : _sign(err));
      } else {
        controlSpeed = err / _dts;  // профиль постоянной скорости
      }
      controlSpeed = constrain(controlSpeed, -_maxSpeed, _maxSpeed);
      controlPos += controlSpeed * _dts;
      controlPos = constrain(controlPos, -_targetPos, _targetPos);
    }
    //return controlPos;
    //Serial.print(encoderValue1);
    //Serial.print(' ');
    // Serial.print(Velocity1);
    // Serial.print(' ');
    // Serial.println();
  }
}

int PIDcalc(long setPoint, long current, float kp, float ki, float kd, float DTS, int dir, bool cutoff, bool off) {  //ПИД для 1 первого мотора

  //if (!off) {

    if (dir == -1) {

      setPoint = -setPoint;
    }


    float Duty = 0;
    long err1 = setPoint - current;
    long deltainput = _previnput - err1;
    _previnput = err1;
    Duty = (float)(err1 * kp);
    integral += (float)err1 * ki * DTS;
    Duty += (float)deltainput * kd / DTS + integral;

    if (cutoff) {  // отсечка (для режимов позиции)
      if (abs(err1) < stopzone) {
        integral = 0;
        Duty = 0;
        // Serial.println("null");
      }
    }

    Duty = constrain(Duty, -255, 255);
    return int(Duty);
    //Serial.println(Duty);
  //}
  // if (Duty == 0) {Serial.println("st");}
  //return Dduty;
  
}

int PIDcalc2(long setPoint, long current, float kp, float ki, float kd, float DTS, int dir, bool cutoff, bool off) {  //ПИД для 2 второго мотора

 // if (!off) {

    if (dir == -1) {

      setPoint = -setPoint;
    }

    int Duty = 0;
    int err = setPoint - current;
    int deltainput = _previnput2 - err;
    _previnput2 = err;
    Duty = err * kp;
    integral2 += (float)err * ki * DTS;
    Duty += (float)deltainput * kd / DTS + integral2;
    Duty = constrain(Duty, -255, 255);

    if (cutoff) {  // отсечка (для режимов позиции)
      if (abs(err) < stopzone2) {
        integral2 = 0;
        Duty = 0;
      }
    }
    //return Dduty;
    return Duty;
  //}

  
}

void movement2(int _Duty, int motor) {

  Dduty = _Duty;


  if (Dduty > 0) {
    //Вращение при регулировании в направлении вперёд по умолчанию
    if (_minDuty != 0) {

      Dduty = Dduty * _k + _minDuty;


    }  // сжимаем диапазон
    //Serial.println(_duty);
  } else {
    // Вращение в обратном напра
    if (_minDuty != 0) { Dduty = Dduty * _k - _minDuty; }  // сжимаем диапазон
  }
  bool stopper = 0, stopper2 = 0;
  int PWM = abs(Dduty);

  if (motor == 1) {

    if ((abs(encoderValue1) + stopzone > _targetPos) && abs(_Duty) < 4) {
      //if (encoderValue1 > _targetPos + stopzone) {stopper = 1;}
      digitalWrite(IN_1, 1);
      digitalWrite(IN_2, 1);
      analogWrite(SPEED_1, 255);
      //Serial.println(Dduty);
      integral = 0;
      stopper = 1;
      //offtrig = 1;
      //delay(100);

    } else {

      if (Dduty > 0) {

        digitalWrite(IN_1, level);
        digitalWrite(IN_2, !level);
        analogWrite(SPEED_1, PWM);
        //Serial.println(PWM);

      } else if (Dduty < 0) {

        digitalWrite(IN_1, !level);
        digitalWrite(IN_2, level);
        analogWrite(SPEED_1, PWM);
      }
      stopper = 0;
    }

  } else if (motor == 2) {

    if ((abs(encoderValue2) + stopzone2 > _targetPos) && abs(_Duty) < 4) {
      //if (encoderValue1 > _targetPos + stopzone) {stopper = 1;}
      digitalWrite(IN_3, 1);
      digitalWrite(IN_4, 1);
      analogWrite(SPEED_2, 255);
      //Serial.println(Dduty);
      integral2 = 0;
      stopper2 = 1;

      //delay(100);

    } else {

      if (Dduty > 0) {

        digitalWrite(IN_3, level);
        digitalWrite(IN_4, !level);
        analogWrite(SPEED_2, PWM);
        //Serial.println(PWM);

      } else if (Dduty < 0) {

        digitalWrite(IN_3, !level);
        digitalWrite(IN_4, level);
        analogWrite(SPEED_2, PWM);
      }
      stopper = 0;
    }
  }
}

void fastwrite() {

  PING |= 0b00100000; //PING PG5 - 4 пин

}

void movement(int _Duty, int motor) {  //Приведение ШИМ к минимальному. Передача сигнала

  if (motor == 1) {}
  Dduty = _Duty;


  if (Dduty > 0) {
    //Вращение при регулировании в направлении вперёд по умолчанию
    if (_minDuty != 0) {

      Dduty = Dduty * _k + _minDuty;


    }  // сжимаем диапазон
    //Serial.println(_duty);
  } else {
    // Вращение в обратном напра
    if (_minDuty != 0) { Dduty = Dduty * _k - _minDuty; }  // сжимаем диапазон
  }

  int PWM = abs(Dduty);

  bool stopper = 0;
  bool stopper2 = 0;

  if ((abs(encoderValue1) + stopzone > _targetPos) && abs(_Duty) < 3) {
    //if (encoderValue1 > _targetPos + stopzone) {stopper = 1;}
    digitalWrite(IN_1, 1);
    digitalWrite(IN_2, 1);
    analogWrite(SPEED_1, 255);
    //Serial.println(Dduty);
    integral = 0;
    stopper = 1;
   // offtrig = 1;

    //delay(100);

  } else {
    stopper = 0;
  }

  if ((abs(encoderValue2) + stopzone2 > _targetPos) && abs(_Duty) < 3) {

    digitalWrite(IN_3, 1);
    digitalWrite(IN_4, 1);
    analogWrite(SPEED_2, 255);
    Serial.println("dt" + String(_Duty));
    Serial.print("motor" + String(motor));
    integral2 = 0;
    stopper2 = 1;
    //delay(100);

  } else {
    stopper2 = 0;
  }

  //if (_targetPos + 100 < encoderValue1) {stopper = 0;}

  if (motor == 1 && stopper != 1) {  //Для первого мотора

    if (Dduty > 0) {

      digitalWrite(IN_1, level);
      digitalWrite(IN_2, !level);
      analogWrite(SPEED_1, PWM);
      //Serial.println(PWM);

    } else {

      digitalWrite(IN_1, !level);
      digitalWrite(IN_2, level);
      analogWrite(SPEED_1, PWM);
    }

  } else if (motor == 2 && stopper2 != 1) {  //Для второго мотора

    if (Dduty > 0) {

      digitalWrite(IN_3, level);
      digitalWrite(IN_4, !level);
      analogWrite(SPEED_2, PWM);

    } else {

      digitalWrite(IN_3, !level);
      digitalWrite(IN_4, level);
      analogWrite(SPEED_2, PWM);

    }  // else if (Dduty == 0) {

    //   digitalWrite(IN_3, level);
    //   digitalWrite(IN_4, level);
    //   analogWrite(SPEED_2, 128);
    // }
  }
}

////////////////////////////////////////////////////////////////////////
/////////////////////////////СЕТЕРЫ SET/////////////////////////////////
void setMillimeters(float millimeters) {

  if (millimeters < 0) {
    dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  _targetPos = (float)((millimeters * (float)ratio) / 0.8);  //0.8 - шаг винта.
  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = abs(_targetPos);
}

void setSpeedMMS(int8_t millimetersSec) {

  _maxSpeed = abs(millimetersSec) * 4172;
  constrain(_maxSpeed, 0, 15000);
  Serial.println("_maxSpeed: " + String(_maxSpeed));
}

void setTarget(long TargetPos) {

  if (TargetPos < 0) {
    dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = abs(TargetPos);
}

void setObor(float ob) {

  dir = 1;
  if (ob < 0) {  //Инверсия направления для отрицательной позиции
    dir = -1;
    Serial.println("Revers");
  }

  Serial.println("Oborotov: " + String(ob));
  _targetPos = (round(ob * ratio));  //ratio - число тиков на оборот
  Serial.println("_targetPos: " + String(_targetPos));

  _targetPos = abs(_targetPos);  //ratio - число тиков на оборот
}

void setDeg(long Deg) {

  if (Deg < 0) {
    dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("Deg: " + String(Deg));
  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = abs(Deg * 8344 / 360);
}

void setRatio(uint16_t _ratio, uint8_t precision) {
  ratio = _ratio / (4 / precision);
  _maxSpeed = _maxSpeed / (4 / precision);
  _accel = _accel / (4 / precision);
}

void setMinDuty(int duty) {

  _minDuty = duty;
  _k = 1.0 - (float)_minDuty / _maxDuty;
}


//////////////////////////////////////////////////////////////////
///////////////////////////ДОПОЛНИТЕЛЬНЫЕ/////////////////////////

void Velocities(int period) {

  // if (millis() - t1 >= period) {

  //   //8344 = 28 * 298 импульсов на оборот
  // t1 = millis();

  //noInterrupts();
  encREF1copy = encREF1;  //int16_t
  encREF2copy = encREF2;  //int16_t
  encREF1 = encREF2 = 0;
  //interrupts();

  Velocity1 = (float)encREF1copy / (float)period;
  //Velocity1 = filter(Velocity1);
  Velocity2 = (float)encREF2copy / (float)period;

  //Serial.println(Velocity1);
  //Serial.println(encREF1copy);
  //controlPosFl = (float)controlPos/ratio;
  //Serial.println(encoderValue2);
  //}
}

void Calculus(int acceleration, int V, long targetPos, float dts) {  //Подсчитывает моменты времени и

  float Xta = (float)targetPos;  //Функция координаты от времени для t;
  float dti = dts;
  float Vmax = (float)V;
  float accel = acceleration;

  tb = Vmax / accel;  //момент времени завершения разгона
  tc = Xta / (Vmax);  //Момент времени завершения равномерного движения

  xb = (float)(Vmax * Vmax) / (2 * accel);  //координата завершения разгона

  float taorig = 0.0;


  if (_accel != 0) {

    if (xb < _targetPos / 2) {  //Если возможна кривая ускорения, равномерного движения и замедления

      taorig = (float)(Xta / Vmax + Vmax / accel);  //Завершающий момент времени 3 сегмента
      curve = 2;                                    //3 сегмента кривой

    } else if (xb > _targetPos / 2 && xb < _targetPos) {  //Ускорение и равномерное движение

      taorig = _targetPos / Vmax + tb / 2;
      tc = taorig;
      curve = 1;

    } else if (xb == _targetPos / 2) {  //Если возможно только ускорение и замедление

      taorig = (float)(2 * (Vmax / accel));
      curve = 0;

    } else if (xb >= _targetPos) {  //Если возможно только ускорение

      taorig = sqrt(2 * _targetPos / accel);
      curve = -1;
    }

  } else {

    taorig = _targetPos / Vmax;
  }

  ta = (int)(round(taorig * 100));
  ta = (float)ta / 100;

  N = round(ta / dti);

  Serial.println("Время ta_Calculus: " + String(ta));
  Serial.println("Время ускорения tb: " + String(tb));
  Serial.println("Позиция после ускорения xb: " + String(xb));

  //T = (0:dti:(ta+dti));
  //X = zeros(N+2, 1);
  Serial.println("Время начала торможения tc: " + String(tc));
  Serial.println("Curve: " + String(curve));
}

void teor(int position, int accel, int V) {

  float Vmax = (float)V;

  if (accel != 0) {

    T_Theoretic = float(((_targetPos - (pow(Vmax, 2) / accel)) / Vmax) + (2 * Vmax / accel));

  } else {

    T_Theoretic = (float)(_targetPos / Vmax);
  }

  Serial.println("T_Theoretic: " + String(T_Theoretic));
  //Serial.println("T_Theoretic = tc: Target/Vmax: " + String((float)_targetPos/Vmax));
}

void ControlPos2() {

  if (millis() - timer >= _dt) {

    f++;
    curTime = f * _dts;
    timer = millis();

    if (_accel != 0 && controlPos2 <= _targetPos) {

      if (curTime <= tb) {

        controlPos2 = _accel * curTime * curTime / 2;
        //Serial.println(controlPos2);

      } else if (curTime > tb && (curTime <= tc || curve == 1)) {  //&& curTime <= tc

        controlPos2 = (_maxSpeed * curTime - xb);  //xb - координата прекращения ускорения
        //Serial.println(controlPos2);

      } else if (curTime > tc && curTime <= ta && curve == 2) {

        controlPos2 = _targetPos - (_accel * ((curTime - tc - tb) * (curTime - tc - tb)) / 2);
        //Serial.println(controlPos2);
        //Serial.println(curTime);
      }  //else if (curTime > ta && curTime < ta+0.10) {controlPos2 = _targetPos;

    } else if (controlPos2 < _targetPos) {

      controlPos2 = _maxSpeed * curTime;
    }

    //PIDcontrol(controlPos2, encoderValue1, true);

    //Serial.println(millis()-timer2);
    //}
    //Serial.println(f);
    //Serial.println(controlPos2);
  }
}

///////////////////////////////////////////////////////////////
/////////////////////СБОР ДАННЫХ И ВЫВОДЫ//////////////////////

void POSITIONS() {  //Вывод позиций столбцами

  Serial.println();
  //Serial.println(_targetPos);
  Serial.print(controlPos);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");
  Serial.print(abs(encoderValue1));
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");

  //Serial.print(_duty2);
  Serial.print(abs(encoderValue2));

  // Serial.print(", ");
  // Serial.print(abs(integral));

  // Serial.print(", ");
  // Serial.print(abs(integral2));


  //Serial.print(controlSpeed/1000);
  //Serial.print(", ");
  //Serial.print(Velocity2);
}

void PWMPORT() {

  Serial.println();

  Serial.print(", ");
  Serial.print(PWM1);

  Serial.print(", ");
  Serial.print(PWM2);

  Serial.print(", ");
  Serial.print(PWM1);
}

void VELS() {  //Вывод скоростей  столбцами

  Serial.println();
  //Serial.println(_targetPos);
  Serial.print(controlSpeed / 1000);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");
  Serial.print(Velocity1);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");

  //Serial.print(_duty2);
  Serial.print(Velocity2);
  //Serial.print(controlSpeed/1000);
  //Serial.print(", ");
  //Serial.print(Velocity2);
}

void update_bufPos(long (&buf)[arr_sizePos1][arr_sizePos2], unsigned long del, int Pos, int pwm, long enc1)  //обновляет буффер значения последнеий аудио сигнал
{
  for (int i = arr_sizePos1 - 2; i >= 0; i--) {

    for (int j = 0; j < arr_sizePos2; j++) {

      buf[i + 1][j] = buf[i][j];
    }

    // buf[i + 1][0] = buf[i][0];
    // buf[i + 1][1] = buf[i][1];
    // buf[i + 1][2] = buf[i][2];
    // buf[i + 1][3] = buf[i][3];
  }

  buf[0][0] = micros() - del;
  buf[0][1] = Pos;
  buf[0][2] = pwm;
  buf[0][3] = enc1;


  //return buf;
}
void update_bufPos_short(long (&buf)[arr_sizePos1][arr_sizePos2], unsigned long del, int Pos, int pwm, long enc1)  //обновляет буффер значения последнеий аудио сигнал
{
  /* for (int i = arr_sizePos1 - 2; i >= 0; i--) {

    for (int j = 0; j < arr_sizePos2; j++) {

      buf[i + 1][j] = buf[i][j];
    }

    // buf[i + 1][0] = buf[i][0];
    // buf[i + 1][1] = buf[i][1];
    // buf[i + 1][2] = buf[i][2];
    // buf[i + 1][3] = buf[i][3];
  }*/

  buf[index_cur][0] = micros() - del;
  buf[index_cur][1] = Pos;
  buf[index_cur][2] = pwm;
  buf[index_cur][3] = enc1;


  //return buf;
}

void update_bufPos_short2(long (&buf)[arr_sizePos1][arr_sizePos1], unsigned long del, int Pos, int pwm, long enc1)  //обновляет буффер значения последнеий аудио сигнал
{
  /* for (int i = arr_sizePos1 - 2; i >= 0; i--) {

    for (int j = 0; j < arr_sizePos2; j++) {

      buf[i + 1][j] = buf[i][j];
    }

    // buf[i + 1][0] = buf[i][0];
    // buf[i + 1][1] = buf[i][1];
    // buf[i + 1][2] = buf[i][2];
    // buf[i + 1][3] = buf[i][3];
  }*/

  buf[index_cur][0] = micros() - del;
  buf[index_cur][1] = Pos;
  buf[index_cur][2] = pwm;
  buf[index_cur][3] = enc1;


  //return buf;
}
void print_bufPos(long (&buf)[arr_sizePos1][arr_sizePos2]) {

  for (int i = arr_sizePos1 - 1; i >= 0; i--) {

    for (int j = 0; j < arr_sizePos2; j++) {

      Serial.print(buf[i][j]);
      Serial.print(" ");
    }

    // Serial.print(buf[i][0]);

    // Serial.print(" ");

    // Serial.print(buf[i][1]);

    // Serial.print(" ");
    // Serial.print(buf[i][2]);

    // Serial.print(" ");
    // Serial.print(buf[i][3]);

    Serial.println(" ");
  }
  Serial.println(";");
}

void print_bufPos_short(long (&buf)[arr_sizePos1][arr_sizePos2]) {

  String mass = "";

  for (int i = 0; i < arr_sizePos1; i++) {

    for (int j = 0; j < arr_sizePos2; j++) {

      mass += String(buf[i][j]) + "\n";
    }
  }
  Serial.println(mass);
}
