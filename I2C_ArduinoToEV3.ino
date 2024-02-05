#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04  // endereço utlizado na comunicação I2C

class Ultrasonic {
private:
  int trigPin, echoPin;  // variáveis que armazenam quais são os pinos do respectivo ultrassônico
public:
  void attach(int trigPin, int echoPin);  // metodo que define os pinos comoo entrada e saída
  float read();                           // método que retorna o valor do ultrasônico
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
  float distance = pulseIn(this->echoPin, HIGH) / 58.2;
  return distance;
};


class LED {
private:
  int pin;  // variável que armazena qual é o pino do respectivo LED
public:
  void attach(int pin);                                  // metodo que define o pino comoo saída
  void toggle(int state = true, bool isDigital = true);  // alterna o estado do LED, podendo ser de 0% a 100% quando analógico e desligado/ligado quando digital
};

void LED::attach(int pin) {
  pinMode(this->pin = pin, OUTPUT);
};

void LED::toggle(int state, bool isDigital) {
  if (isDigital)
    digitalWrite(this->pin, state ? HIGH : LOW);
  else
    analogWrite(this->pin, map(state, 0, 100, 0, 255));

  //Serial.println("LED[" + String(this->pin) + "] Turned on/off");
  //Serial.println("LED[" + String(this->pin) + "] at " + String(map(state, 0, 100, 0, 255)) + "%");
}


class InfraRed {
private:
  int pin;  // variável que armazena qual é o pino do respectivo sensor infra vermelho
public:
  void attach(int pin, boolean isDigital);  // metodo que define o pino comoo entrada caso seja digital
  float getValue(bool isDigital);           // método que retorna o valor do sensor infra vermelho
};

void InfraRed::attach(int pin, boolean isDigital = true) {
  if (isDigital)
    pinMode(pin, INPUT);
  this->pin = pin;
};

float InfraRed::getValue(bool isDigital) {
  if (isDigital)
    return digitalRead(this->pin);
  else
    return map(analogRead(this->pin), 0, 1023, 0, 100);
};


Servo servo[6];

typedef enum {
  _LEFT_ARM_SERVO,
  _LEFT_CLAW_SERVO,
  _RIGHT_CLAW_SERVO,
  _CONTAINER_SERVO,
} servos_index;

Ultrasonic ultrasonic[6];
LED led[6];
InfraRed infraRed[3];

void setup() {
  Wire.begin(SLAVE_ADDRESS);    // define o endereço para comunicação I2C e o inicia
  Wire.onReceive(receiveData);  // irá chamar a função 'receiveData' ao receber informação através do I2C
  Wire.onRequest(sendData);     // após receber, irá requisitar que envie algo, e será chamado a função 'sendData'

  //servo[ID].attach(pin);
  //servo[_LEFT_ARM_SERVO].attach(5);
  //ultrasonic[ID].attach(pin);
  //led[ID].attach(pin)
  //infraRed[ID].attach(pin);

  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Ready!");
}

void loop() {
  delay(500);
}


typedef enum {
  _MAIN_ACTION,       // ação principal
  _SECONDARY_ACTION,  // ação secundária
  _ACTION_3,          // terceira ação
  _ACTION_4,          // quarta ação
  _ID,                // identificador
} instruction_index;

typedef enum {  // index das ações utilizado nas listas
  _USE_LED,
  _USE_SERVO,
  _USE_STEPPER_MOTOR,
  _USE_INFRA_RED,
  _USE_ULTRASONIC,
} options_index;


uint8_t instruction[] = { 255, 0, 0, 0, 0, 0, 0, 0 };  // valores recebidor por I2C
uint8_t returnValue[] = { 255, 0, 0, 0, 0, 0, 0, 0 };  // valores que serão retornados por I2C
boolean pendingValue;                                  // se possui algum valor para ser enviado

void receiveData(int bytesIn) {
  for (int byte_count = 0; 1 < Wire.available(); byte_count++) {
    instruction[byte_count] = Wire.read();
  }
  const byte dummyByte = Wire.read();                // lê o último byte fictício (não tem significado, mas precisa ser lido)
  const boolean isDigital = instruction[_ID] < 100;  // se o ID for maior que 100, significa que é analógico
  pendingValue = false;                              // reseta a variável responsável por identificar se há dados a serem enviados

  if (instruction[_MAIN_ACTION] == 255) return;

  switch (instruction[_MAIN_ACTION]) {
    case _USE_LED:
      led[instruction[_ID]].toggle(isDigital, instruction[_SECONDARY_ACTION]);
      break;

    case _USE_SERVO:
      servo[instruction[_ID]].write(instruction[_SECONDARY_ACTION]);
      //Serial.println("Changed the angle of the Servo Motor[" + String(instruction[_ID]) + "] to " + String(instruction[_SECONDARY_ACTION]) + "degrees");
      break;

    case _USE_STEPPER_MOTOR:

      break;
    case _USE_INFRA_RED:
      returnValue[0] = infraRed[instruction[_ID]].getValue(isDigital);
      //Serial.println("Infra Red[" + String(instruction[_ID]) + "] = " + String(returnValue[0]));
      pendingValue = true;
      break;

    case _USE_ULTRASONIC:
      returnValue[0] = ultrasonic[_ID].read();
      //Serial.println("Ultrasonic[" + String(instruction[_ID]) + "] = " + String(returnValue[0]));
      pendingValue = true;
      break;
  }
}

void sendData() {
  if (pendingValue)
    Wire.write(returnValue[0]);  // Envia o valor obtido pelo sensor
}