#include <P1AM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <TimeEvent.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MemoryFree.h>;

/////////////////////////////////////////////////////
//UTILIDADES
//MemoryFree Frame
unsigned long frame=0;
unsigned long beforeFrame = 0;
TimeEvent frameEvent = TimeEvent(1000);
TimeEvent dseReconnect = TimeEvent(1000);
//////////////////////////////////////////////////////
//CONFIGURACION ETHERNET-MODBUS
byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0x68, 0x98};//P1AM-ETH mac
IPAddress ip(192, 168, 137, 177);

#define NUMBER_OF_DSE 2
int dseHR[NUMBER_OF_DSE][37];
bool dseAlarms[NUMBER_OF_DSE][150];
bool dseErrorComm[NUMBER_OF_DSE];

EthernetClient modules[NUMBER_OF_DSE];
ModbusTCPClient modbusTCPClient[NUMBER_OF_DSE]={
  ModbusTCPClient(modules[0]),
  ModbusTCPClient(modules[1])
};

IPAddress servers[NUMBER_OF_DSE]={//IP Addresses of the Servers
  IPAddress(192, 168, 137,  128),
  IPAddress(192, 168, 137,  126)
};

#define NUMBER_OF_MODBUS_CLIENTS 8
EthernetServer server(502);
ModbusTCPServer modbusTCPServer;
EthernetClient clients[NUMBER_OF_MODBUS_CLIENTS];
int client_cnt=0;
/////////
//SETUP//
/////////
void setup(){
  ///////////////////////
  //UTILIDADES
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("INIT");
  frameEvent.repeat();
  frameEvent.start();
  dseReconnect.repeat();
  dseReconnect.start();

  //////////////////////
  //ETHERNET-MODBUS
  Ethernet.begin(mac, ip);
  modbusTCPClient[0].setTimeout(100);
  modbusTCPClient[1].setTimeout(100);

  server.begin();
  if(!modbusTCPServer.begin()){
    Serial.println("Failed to start Modbus TCP server");
    while(1);
  }else{
    Serial.println("Modbus TCP server initialized");
  }
  //modbusTCPServer.configureCoils(0x00,300);
  modbusTCPServer.configureDiscreteInputs(0x00,NUMBER_OF_DSE*150);
  modbusTCPServer.configureHoldingRegisters(0x00,NUMBER_OF_DSE*37);
  // modbusTCPServer.configureInputRegisters(0x00,16);
  initializeArrays();

}//FIN SETUP

////////
//LOOP//
////////
void loop(){

  readDseAlarms();
  computeDseAlarms();
  updateInputs();
  updateHoldingRegisters();
  handleModbusClients();

  /////////////////////////////
  //UTILIDADES
  updateFrame();
  if(frameEvent.run()){
    Serial.print("Frame: ");
    Serial.println(frame);
    printMemory();
    Serial.print("DSE Errors: ");
    for(int i=0; i<NUMBER_OF_DSE;i++){
      Serial.print(dseErrorComm[i]);
      Serial.print(" ");
    }
    Serial.println();

  }

  if(dseReconnect.run()){
    for(int i=0; i<NUMBER_OF_DSE; i++){
      dseErrorComm[i]=false;
    }
  }
}//FIN LOOP

////////////////////////////////////////////////
void computeDseAlarms(){
  //esta funcion separa las 4 alarmas que vienen dentro del mismo registro
  //cada alarma es de 4 bits y solo se busca que este entre los valores de 2 a 4
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){

      int a1 = dseHR[i][j]&0b00001111;
      int a2 = (dseHR[i][j]>>4)&0b00001111;
      int a3 = (dseHR[i][j]>>8)&0b00001111;
      int a4 = (dseHR[i][j]>>12)&0b00001111;

      dseAlarms[i][j*4]=a1<=4 && a1>=2;
      dseAlarms[i][(j*4)+1]=a2<=4 && a2>=2;
      dseAlarms[i][(j*4)+2]=a3<=4 && a3>=2;
      dseAlarms[i][(j*4)+3]=a4<=4 && a4>=2;
    }
  }
}

void updateInputs() { //Write the Input bits to the Modbus Library
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<150;j++){
      modbusTCPServer.discreteInputWrite((i*150)+j,dseAlarms[i][j]);
    }
  }
}

void updateHoldingRegisters() {//Read the Holding Register words from the Modbus Library
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      modbusTCPServer.holdingRegisterWrite((i*37)+j,dseHR[i][j]);
    }
  }
}

void handleModbusClients(){
  EthernetClient newClient = server.accept(); //listen for incoming clients
  if (newClient) { //process new connection if possible
  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) { //Eight connections possible, find first available.
    if (!clients[i]) {
      clients[i] = newClient;
      client_cnt++;
      Serial.print("Client Accept:"); //a new client connected
      Serial.print(newClient.remoteIP());
      Serial.print(" , Total:");
      Serial.println(client_cnt);
      break;
      }
    }
  }
  //If there are packets available, receive them and process them.
  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) {
    if (clients[i].available()) { //if data is available
      modbusTCPServer.accept(clients[i]); //accept that data
      modbusTCPServer.poll();// service any Modbus TCP requests, while client connected
      }
  }
  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) { // Stop any clients which are disconnected
    if (clients[i] && !clients[i].connected()) {
      clients[i].stop();
      client_cnt--;
      Serial.print("Client Stopped, Total: ");
      Serial.println(client_cnt);
    }
  }
}

void readDseAlarms(){
  //esta funcion lee los registros de alarma de los DSE
  for(int d=0;d<NUMBER_OF_DSE;d++){
    if(dseErrorComm[d]){
      continue;
    }
    if (!modbusTCPClient[d].connected()) {// client not connected, start the Modbus TCP client
      Serial.print("Attempting to connect to Modbus TCP server at IP:");
      Serial.println(servers[d]);
      if (!modbusTCPClient[d].begin(servers[d])) {
        Serial.println("Modbus TCP Client failed to connect!");
        dseErrorComm[d]=true;
      } else {
        Serial.println("Modbus TCP Client connected");
        dseErrorComm[d]=false;
      }
    } else {
      if(modbusTCPClient[d].requestFrom(HOLDING_REGISTERS,39425,37)){
        if(modbusTCPClient[d].available()){
          for(int i=0;i<37;i++){
            dseHR[d][i] = modbusTCPClient[d].read();
          }
        }
      }else{
        Serial.println(modbusTCPClient[d].lastError());
        dseErrorComm[d]=true;
      }
    }
  }
}

void initializeArrays(){
    for(int i=0;i<NUMBER_OF_DSE;i++){
      for(int j=0;j<150;j++){
        dseAlarms[i][j]=0;
      }
    }
    for(int i=0;i<NUMBER_OF_DSE;i++){
      for(int j=0;j<37;j++){
        dseHR[i][j]=false;
      }
    }
    for(int i=0; i<NUMBER_OF_DSE; i++){
      dseErrorComm[i]=false;
    }
}

void printMemory(){
  Serial.print(F("MEM FREE: "));
  Serial.println(freeMemory(), DEC);
}

void updateFrame(){
  frame = micros() - beforeFrame;
  beforeFrame = micros();
}
