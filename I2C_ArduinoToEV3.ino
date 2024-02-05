#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04

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


Servo servo[6];
Ultrasonic ultrasonic[6];

void setup() {
  Wire.begin(SLAVE_ADDRESS);    // Define o endereço I2C e o inicia
  Wire.onReceive(receiveData);  // Irá chamar a função 'receiveData' ao receber informação através do I2C
  Wire.onRequest(sendData);     // Após receber, irá requisitar que envie algo, e será chamado a função 'sendData'

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
  _FIRST_PIN,    // Primeiro pino
  _SECOND_PIN,   // Segundo pino
  _ARE_DIGITAL,  // Pinos em porta digital?
  _ACTION_1,     // Tipo de ação (led, servo, sensor, motor)
  _ACTION_2,     // Segunda ação
  _ACTION_3,     // Terceira ação
  _ACTION_4,     // Quarta ação
  _ACTION_5,     // Quinta ação
} instruction_index;

typedef enum {  // Index utilizado nas listas
  _INITIALIZE_PINS = (0),
  _LED = (1),
  _SERVO = (2),
  _MOTOR = (3),
  _SENSOR = (4),
  _INFRA_RED = (5),
  _ULTRASONIC = (6),
} options_index;


uint8_t instruction[] = { 255, 0, 0, 0, 0, 0, 0, 0 };  // Valores recebidor por I2C
uint8_t sensorValue[] = { 000, 0, 0, 0, 0, 0, 0, 0 };  // Valores que serão retornados por I2C


void receiveData(int bytesIn) {
  for (int byte_count = 0; 1 < Wire.available(); byte_count++) {
    instruction[byte_count] = Wire.read();
  }
  byte dummyByte = Wire.read();
  // Lê o último byte fictício (não tem significado, mas precisa ser lido)

  Serial.print("ACTION: ");
  switch (instruction[_ACTION_1]) {
    // _ACTION_1 -> Tipo de ação
    case _INITIALIZE_PINS:
      Serial.print("Initialized pin(s) " + String(instruction[_FIRST_PIN]));
      switch (instruction[_ACTION_2]) {
        // _ACTION_2 -> Tipo de sensor
        case _LED:
          pinMode(instruction[_FIRST_PIN], OUTPUT);
          Serial.println(" as a LED");
          break;

        case _SERVO:
          servo[instruction[_ACTION_3]].attach(instruction[_FIRST_PIN]);
          Serial.println(" as a Servo[" + String(instruction[_ACTION_3]) + "]");
          // _ACTION_3 -> ID do servo
          break;

        case _MOTOR:
          pinMode(instruction[_FIRST_PIN], OUTPUT);
          Serial.println(" as a First Motor");
          pinMode(instruction[_SECOND_PIN], OUTPUT);
          Serial.println(String(instruction[_SECOND_PIN]) + " as a Second Motor");
          break;

        case _SENSOR:
          switch (instruction[_ACTION_3]) {
            // _ACTION_3 ->  Tipo de sensor
            case _INFRA_RED:
              if (instruction[_ARE_DIGITAL])
                pinMode(instruction[_FIRST_PIN], INPUT);
              Serial.println(" as a Infra Red sensor");
              break;

            case _ULTRASONIC:
              ultrasonic[instruction[_ACTION_4]].attach(instruction[_FIRST_PIN], instruction[_SECOND_PIN]);
              Serial.println("and" + String(instruction[_SECOND_PIN]) + "as a Ultrasonic Sensor [" + String(instruction[_ACTION_4]) + "]");
              // _ACTION_4 -> ID do ultrasonico
              break;
          }
          break;
      }
      break;

    case _LED:
      digitalWrite(instruction[_FIRST_PIN], instruction[_ACTION_2] ? HIGH : LOW);
      Serial.println("Turned on/off the LED[" + String(instruction[_FIRST_PIN]) + "]");
      // _ACTION_2 -> Estado do LED (Ligado/Desligado)
      break;

    case _SERVO:
      servo[instruction[_FIRST_PIN]].write(instruction[_ACTION_2]);
      Serial.println("Changed the angle of the Servo Motor[" + String(instruction[_FIRST_PIN]) + "] to " + String(instruction[_ACTION_2]) + "degrees");
      // _ACTION_2 -> Ângulo do servo motor
      break;

    case _MOTOR:
      analogWrite(instruction[_FIRST_PIN], instruction[_ACTION_2] * 2.55);
      Serial.println("Changed the velocity of the DC Motor[" + String(instruction[_FIRST_PIN]) + "] to " + String(instruction[_ACTION_2]));
      // _ACTION_2 -> Velocidade do primeiro motor
      analogWrite(instruction[_SECOND_PIN], instruction[_ACTION_3] * 2.55);
      Serial.println("Changed the velocity of the DC Motor[" + String(instruction[_SECOND_PIN]) + "] to " + String(instruction[_ACTION_3]));
      // _ACTION_3 -> Velocidade do segundo motor
      break;

    case _SENSOR:
      Serial.print("Read sensor value of ");
      switch (instruction[_ACTION_2]) {
        // _ACTION_2 -> Tipo de sensor
        case _INFRA_RED:
          sensorValue[0] = instruction[_ARE_DIGITAL] ? digitalRead(instruction[_FIRST_PIN]) : analogRead(instruction[_FIRST_PIN]);
          Serial.println("Infra Red[" + String(instruction[_FIRST_PIN]) + "] = " + String(sensorValue[0]));
          break;

        case _ULTRASONIC:
          sensorValue[0] = ultrasonic[_ACTION_3].read();
          Serial.println("Ultrasonic[" + String(instruction[_ACTION_3]) + "] = " + String(sensorValue[0]));
          // _ACTION_3 -> ID do ultrasonico
          break;
      }
      break;
  }
}

void sendData() {
  if (instruction[_ACTION_1] == _SENSOR) {
    // _ACTION_1 -> Tipo de ação
    Wire.write(sensorValue[0]);  // Envia o valor obtido pelo sensor
  }
}