#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04  // endereço utlizado na comunicação I2C

class Ultrasonic {
private:
  int trigPin, echoPin;  // variáveis que armazenam quais são os pinos do respectivo ultrassônico
  int timeOut = 5000;    // tempo máximo do pulso da medição. A partir de uma certa distância ocasiona um bug na comunicação I2C
public:
  void attach(int trigPin, int echoPin);         // metodo que define os pinos como entrada e saída
  float read();                                  // método que retorna o valor do ultrasônico
  bool compare(int operatorNumber, int value2);  // método que lê o valor do ultrassônico e retorna o resultado da comparação com outro valor
};

void Ultrasonic::attach(int trigPin, int echoPin) {
  pinMode(this->trigPin = trigPin, OUTPUT);
  pinMode(this->echoPin = echoPin, INPUT);
};

float Ultrasonic::read() {
  digitalWrite(this->trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trigPin, LOW);
  float distance = pulseIn(this->echoPin, HIGH, this->timeOut) / 58.2;
  return distance;
};

bool Ultrasonic::compare(int operatorNumber, int value2) {
  int value1 = this->read();
  switch (operatorNumber) {
    case 0: return value1 == value2; break;
    case 1: return value1 != value2; break;
    case 2: return value1 > value2; break;
    case 3: return value1 >= value2; break;
    case 4: return value1 < value2; break;
    case 5: return value1 <= value2; break;
    default: return false;
  }
};


class LED {
private:
  int pin;
  bool isDigital;
public:
  void attach(int pin, bool isDigital);  // metodo que define o pino comoo saída
  void toggle(int state = true);         // alterna o estado do LED
};

void LED::attach(int pin, bool isDigital = true) {
  if (isDigital) {
    pinMode(this->pin = pin, OUTPUT);
    this->isDigital = true;
  }
};

void LED::toggle(int state) {
  if (this->isDigital)
    digitalWrite(this->pin, state ? HIGH : LOW);
  else
    analogWrite(this->pin, map(state, 0, 100, 0, 255));
};


class InfraRed {
private:
  int pin;
  bool isDigital;
public:
  void attach(int pin, bool isDigital);  // metodo que define o pino como entrada caso seja digital
  float read();                          // método que retorna o valor do sensor infra vermelho
};

void InfraRed::attach(int pin, bool isDigital = true) {
  if (isDigital) {
    this->isDigital = true;
    pinMode(pin, INPUT);
  }
  this->pin = pin;
};

float InfraRed::read() {
  if (this->isDigital)
    return digitalRead(this->pin);
  else
    return map(analogRead(this->pin), 0, 1023, 0, 10);
};


class Button {
private:
  int pin;
  bool isDigital;
public:
  void attach(int pin, bool isDigital);  // metodo que define o pino como entrada caso seja digital
  int read();                            // método que retorna o valor do botão
};

void Button::attach(int pin, bool isDigital = true) {
  if (isDigital) {
    pinMode(pin, INPUT);
    this->isDigital = true;
  }
};

int Button::read() {
  if (this->isDigital) {
    return digitalRead(this->pin);
  } else {
    return map(analogRead(this->pin), 0, 1023, 0, 100);
  }
};

/* ----------------------------------------------------------------- */

const typedef enum {
  _LEFT_ARM_SERVO,
  _RIGHT_ARM_SERVO,
  _LEFT_CLAW_SERVO,
  _RIGHT_CLAW_SERVO,
  _CONTAINER_SERVO,
  _SERVO_LENGTH,
} servos_index;

const typedef enum {
  _FRONTAL_ULTRASONIC,
  _SIDE_ULTRASONIC,
  _ULTRASONIC_LENGTH,
} ultrasonics_index;

const typedef enum {
  _LED1,
  _LED2,
  _LED_LENGTH,
} leds_index;

const typedef enum {
  _DETECTOR_IR,
  _VERIFIER_IR,
  _IR_LENGTH,
} infrareds_index;

const typedef enum {
  _BUMPER_BUTTON,
  _BUTTON_LENGTH,
} buttons_index;

Servo servo[_SERVO_LENGTH];
Ultrasonic ultrasonic[_ULTRASONIC_LENGTH];
LED led[_LED_LENGTH];
InfraRed infraRed[_IR_LENGTH];
Button button[_BUTTON_LENGTH];


void setup() {
  Wire.begin(SLAVE_ADDRESS);    // define o endereço para comunicação I2C e o inicia
  Wire.onReceive(receiveData);  // recebe dados através do I2C (a função 'receiveData' será executada toda vez que for utilizado o bloco 'Write')
  Wire.onRequest(requestData);  // envia dados através do I2C (a função 'requestData' será executada toda vez que for utilizado o bloco 'Read')

  servo[_LEFT_ARM_SERVO].attach(9, 0, 90);
  //servo[_RIGHT_ARM_SERVO].attach();
  servo[_LEFT_CLAW_SERVO].attach(10, 90 - 55, 90);
  servo[_RIGHT_CLAW_SERVO].attach(11), 83, 83 + 55;
  //servo[_CONTAINER_SERVO].attach();

  ultrasonic[_FRONTAL_ULTRASONIC].attach(3, 4);
  //ultrasonic[_SIDE_ULTRASONIC].attach();

  led[_LED1].attach(7);
  //led[_LED2].attach();

  infraRed[_DETECTOR_IR].attach(0, true);
  infraRed[_VERIFIER_IR].attach(1, false);

  //button[_BUMPER_BUTTON].attach();

  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  led[_LED1].toggle(true);
  Serial.println("Ready!");
}

void loop() {
  delay(500);
}


typedef enum {
  _MAIN_ACTION,  // ação principal
  _ACTION_2,     // ação secundária
  _ACTION_3,     // terceira ação
  _VALUE_1,      // primeiro valor
  _VALUE_2,      // segundo valor
  _ID_1,         // primeiro identificador
  _ID_2,         // segundo identificador
} instruction_index;

typedef enum {  // index das ações principais
  _USE_LED,
  _USE_SERVO_DEGREE,
  _USE_SERVO_PERCENTAGE,
  _USE_SERVO_TO_MAX,
  _USE_TWO_SERVOS_DEGREE,
  _USE_TWO_SERVOS_PERCENTAGE,
  _USE_TWO_SERVOS_TO_MAX,
  _USE_STEPPER_MOTOR,
  _USE_INFRA_RED,
  _USE_ULTRASONIC,
  _USE_COMPARE_ULTRASONIC,
  _USE_BUTTON,
} options_index;


uint8_t instruction[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // valores recebidos por I2C
uint8_t returnValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // valores que serão enviados por I2C
boolean pendingValue;                                // se possui algum valor para ser enviado
int fragmentIndex;                                   // variável utilizada para identificar os fragmentos
boolean ignoreFirstSend;                             // usada para ignorar o envio automático


void receiveData(int bytesIn) {
  for (int byte_count = 0; 1 < Wire.available(); byte_count++) {  // recebe os valores por I2C
    instruction[byte_count] = Wire.read();
  }
  const byte dummyByte = Wire.read();  // lê o último byte fictício (não tem significado, mas precisa ser lido)
  pendingValue = false;                // reseta a variável responsável por identificar se há dados a serem enviados
  fragmentIndex = 0;                   // reseta a variável responsável por guardar a quantidade de fragmentos à serem retornados

  switch (instruction[_MAIN_ACTION]) {
    case _USE_LED:
      led[instruction[_ID_1]].toggle(instruction[_VALUE_1]);
      break;

    case _USE_SERVO_DEGREE:
      servo[instruction[_ID_1]].write(instruction[_VALUE_1]);
      break;

    case _USE_SERVO_PERCENTAGE:
      servo[instruction[_ID_1]].write(map(instruction[_VALUE_1], 0, 100, 0, 180));
      break;

    case _USE_SERVO_TO_MAX:
      servo[instruction[_ID_1]].write(instruction[_VALUE_1] ? 180 : 0);
      break;

    case _USE_TWO_SERVOS_DEGREE:
      servo[instruction[_ID_1]].write(instruction[_VALUE_1]);
      servo[instruction[_ID_2]].write(instruction[_VALUE_2]);
      break;

    case _USE_TWO_SERVOS_PERCENTAGE:
      servo[instruction[_ID_1]].write(map(instruction[_VALUE_1], 0, 100, 0, 180));
      servo[instruction[_ID_2]].write(map(instruction[_VALUE_2], 0, 100, 0, 180));
      break;

    case _USE_TWO_SERVOS_TO_MAX:
      servo[instruction[_ID_1]].write(instruction[_VALUE_1] ? 180 : 0);
      servo[instruction[_ID_2]].write(instruction[_VALUE_2] ? 180 : 0);
      break;

    case _USE_STEPPER_MOTOR:

      break;

    case _USE_INFRA_RED:
      returnValue[0] = infraRed[instruction[_ID_1]].read();
      pendingValue = true;
      break;

    case _USE_ULTRASONIC:
      int value = constrain(round(ultrasonic[instruction[_ID_1]].read() * 10) / 10, 0, 60);

      for (int v = value; (v /= 8) >= 1; fragmentIndex++) {}  // calcula quantos digitos o número convertido vai ter

      for (int i = fragmentIndex; value; i--) {  // faz a conversão de base decimal para octal
        returnValue[i] = value % 8;
        value /= 8;
      }

      fragmentIndex = instruction[_ACTION_2] - 1;  // quantidade de digitos que o usuário quer que retorne

      pendingValue = true;
      break;

    case _USE_COMPARE_ULTRASONIC:
      returnValue[0] = ultrasonic[instruction[_ID_1]].compare(instruction[_ACTION_2], instruction[_VALUE_1]);
      pendingValue = true;
      break;

    case _USE_BUTTON:
      returnValue[0] = button[instruction[_ID_1]].read();
      pendingValue = true;
      break;
  }

  ignoreFirstSend = true;
}

void requestData() {
  if (pendingValue) {
    if (ignoreFirstSend) {
      ignoreFirstSend = false;
      return;
    }

    returnValue[fragmentIndex] = constrain(returnValue[fragmentIndex], 0, 7);  // evita que o valor seja maior que 7
    Wire.write(returnValue[fragmentIndex]);                                    // envia o valor por I2C
    returnValue[fragmentIndex] = 0;

    if (fragmentIndex > 0)
      fragmentIndex--;
  }
}