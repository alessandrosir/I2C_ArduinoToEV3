#include <Wire.h>

#define SLAVE_ADDRESS 0x04  // endereço utlizado na comunicação I2C

uint8_t instruction[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // valores recebidos por I2C
uint8_t returnValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // valores que serão enviados por I2C
boolean pendingValue;                                // se possui algum valor para ser enviado
boolean ignoreFirstSend;                             // usada para ignorar o envio automático

void setup() {
  Wire.begin(SLAVE_ADDRESS);    // define o endereço para comunicação I2C e o inicia
  Wire.onReceive(receiveData);  // irá chamar a função 'receiveData' ao receber informação através do I2C
  Wire.onRequest(sendData);     // após receber, irá requisitar que envie algo, e será chamado a função 'sendData'
}

void loop() {
  delay(500);
}


void receiveData(int bytesIn) {
  for (int byte_count = 0; 1 < Wire.available(); byte_count++) {  // recebe os valores por I2C
    instruction[byte_count] = Wire.read();
  }
  const byte dummyByte = Wire.read();  // lê o último byte fictício (não tem significado, mas precisa ser lido)
  pendingValue = false;                // reseta a variável responsável por identificar se há dados a serem enviados



  ignoreFirstSend = true;
}

void sendData() {
  if (pendingValue) {
    if (ignoreFirstSend) {
      ignoreFirstSend = false;
      return;
    }



    Wire.write(returnValue[0]);  // envia o valor por I2C
    returnValue[0] = 0;
  }
}