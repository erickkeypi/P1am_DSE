#include <P1AM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <TimeEvent.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MemoryFree.h>;


byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0x68, 0x98};//P1AM-ETH mac
IPAddress ip(192, 168, 137, 177);

EthernetClient clients[2];
ModbusTCPClient modbusTCPClient[2]={
  ModbusTCPClient(clients[0]),
  ModbusTCPClient(clients[1])
};

IPAddress servers[2]={//IP Addresses of the Servers
  IPAddress(192, 168, 137,  128),
  IPAddress(192, 168, 137,  126)
};

//MemoryFree Frame
unsigned long frame=0;
unsigned long beforeFrame = 0;


void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("INIT");

  Ethernet.begin(mac, ip);
  modbusTCPClient[0].setTimeout(500);
  modbusTCPClient[1].setTimeout(500);


}

void loop(){
  if (!modbusTCPClient[0].connected()) {// client not connected, start the Modbus TCP client
    Serial.print("Attempting to connect to Modbus TCP server at IP:");
    Serial.println(servers[0]);
    if (!modbusTCPClient[0].begin(servers[0])) {
      Serial.println("Modbus TCP Client failed to connect!");
    } else {
      Serial.println("Modbus TCP Client connected");
    }
  } else {
    Serial.println("Client[0] conected");
  }
  if (!modbusTCPClient[1].connected()) {// client not connected, start the Modbus TCP client
    Serial.print("Attempting to connect to Modbus TCP server at IP:");
    Serial.println(servers[1]);
    if (!modbusTCPClient[1].begin(servers[1])) {
      Serial.println("Modbus TCP Client failed to connect!");
    } else {
      Serial.println("Modbus TCP Client connected");
    }
  } else {
    Serial.println("Client[1] conected");
  }
  updateFrame();
  Serial.print("Frame: ");
  Serial.println(frame);
  printMemory();
  delay(1000);

}

void printMemory(){
  Serial.print(F("MEM FREE: "));
  Serial.println(freeMemory(), DEC);
}

void updateFrame(){
  frame = micros() - beforeFrame;
  beforeFrame = micros();
}
