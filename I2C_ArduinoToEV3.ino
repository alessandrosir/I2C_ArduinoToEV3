#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04  // endereço da comunicação I2C

class Ultrasonic {
private:
  int trigPin, echoPin;
public:
  void attach(int trigPin, int echoPin);

  float read() {
    digitalWrite(this->trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigPin, LOW);
    float distance = pulseIn(this->echoPin, HIGH) / 58.2;
    return distance;
  }
};

void Ultrasonic::attach(int trigPin, int echoPin) {
  pinMode(this->trigPin = trigPin, OUTPUT);
  pinMode(this->echoPin = echoPin, INPUT);
};


class LED {
public:
  void attach(int pin);
};

void LED::attach(int pin) {
  pinMode(pin, OUTPUT);
};


class InfraRed {
public:
  void attach(int pin, boolean isDigital);
};

void InfraRed::attach(int pin, boolean isDigital = true) {
  if (isDigital)
    pinMode(pin, INPUT);
};



Servo servo[6];
Ultrasonic ultrasonic[6];
//LED led[6];
//InfraRed infraRed[3];

void setup() {
  Wire.begin(SLAVE_ADDRESS);    // define o endereço I2C e o inicia
  Wire.onReceive(receiveData);  // irá chamar a função 'receiveData' ao receber informação através do I2C
  Wire.onRequest(sendData);     // após receber, irá requisitar que envie algo, e será chamado a função 'sendData'

  //servo[ID].attach(pin);
  //ultrasonic[ID].attach(pin);
  //led[ID].attach(pin);
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
  const byte dummyByte = Wire.read();                                           // lê o último byte fictício (não tem significado, mas precisa ser lido)
  const boolean areDigital = instruction[_ID] < 100 || instruction[_ID] < 100;  // se alguns dos IDs forem maior que 100, significa que são analógicos
  pendingValue = false;

  if (instruction[_MAIN_ACTION] == 255) return;

  switch (instruction[_MAIN_ACTION]) {
    case _USE_LED:
      if (areDigital) {
        digitalWrite(instruction[_ID], instruction[_SECONDARY_ACTION] ? HIGH : LOW);
        Serial.println("LED[" + String(instruction[_ID]) + "] Turned on/off");
      } else {
        instruction[_ID] -= 100;
        analogWrite(instruction[_ID], map(instruction[_SECONDARY_ACTION], 0, 100, 0, 255));
        Serial.println("LED[" + String(instruction[_ID]) + "] at " + String(map(instruction[_SECONDARY_ACTION], 0, 100, 0, 255)) + "%");
      }
      break;

    case _USE_SERVO:
      servo[instruction[_ID]].write(instruction[_SECONDARY_ACTION]);
      Serial.println("Changed the angle of the Servo Motor[" + String(instruction[_ID]) + "] to " + String(instruction[_SECONDARY_ACTION]) + "degrees");
      break;

    case _USE_STEPPER_MOTOR:

      break;
    case _USE_INFRA_RED:
      returnValue[0] = instruction[areDigital] ? digitalRead(instruction[_ID]) : map(analogRead(instruction[_ID]), 0, 1023, 0, 100);
      Serial.println("Infra Red[" + String(instruction[_ID]) + "] = " + String(returnValue[0]));
      pendingValue = true;
      break;

    case _USE_ULTRASONIC:
      returnValue[0] = ultrasonic[_ID].read();
      Serial.println("Ultrasonic[" + String(instruction[_ID]) + "] = " + String(returnValue[0]));
      pendingValue = true;
      break;
  }
}

void sendData() {
  if (pendingValue)
    Wire.write(returnValue[0]);  // Envia o valor obtido pelo sensor
}