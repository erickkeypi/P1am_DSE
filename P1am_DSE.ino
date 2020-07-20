//las alarmas estan en los inputs
//los bits que utiliza la cmore estan en los coils

//cada 30 dias se debe activar un coil para que la cmore actualice las fechas de los DSE
//se necesita registros para guardar la prioridad de los gen y un coil para el error de prioridad
#include <P1AM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <TimeEvent.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MemoryFree.h>;

#define NUMBER_OF_DSE 2
#define NUMBER_OF_MODBUS_CLIENTS 8
#define MODBUS_TIMEOUT 200
#define MODBUS_RECONNECT_TIME 30000
#define UPDATE_DATE_PERIOD 86400000
/////////////////////////////////////////////////////
//UTILIDADES
//MemoryFree Frame
unsigned long frame=0;
unsigned long beforeFrame = 0;
TimeEvent frameEvent = TimeEvent(1000);
TimeEvent updateDateEvent = TimeEvent(UPDATE_DATE_PERIOD);

//////////////////////////////////////////////////////
//RECONNECT
TimeEvent dseReconnect = TimeEvent(MODBUS_RECONNECT_TIME);
//////////////////////////////////////////////////////
//CONFIGURACION ETHERNET-MODBUS
byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0x68, 0x98};//P1AM-ETH mac
IPAddress ip(192, 168, 137, 177);

int dseIR[NUMBER_OF_DSE][37];//alarmas leidas
bool dseAlarms[NUMBER_OF_DSE][150];//bits de las alarmas
bool dseErrorComm[NUMBER_OF_DSE];//error de comunicacion

bool updateModulesDates = false;
bool gensetPriorityError =false;

bool busLive = false;
bool gen1Breaker = false;
bool gen2Breaker = false;
bool gen3Breaker = false;
bool gen4Breaker = false;
bool master1BusAvailable = false;
bool master2BusAvailable = false;
bool master3BusAvailable = false;
bool master4BusAvailable = false;


bool generalCommonAlarm = false;
bool gen1CommonAlarm =false;
bool gen2CommonAlarm =false;
bool gen3CommonAlarm =false;
bool gen4CommonAlarm =false;
bool master1CommonAlarm = false;
bool master2CommonAlarm = false;
bool master3CommonAlarm = false;
bool master4CommonAlarm = false;

unsigned int gen1Priority =0;
unsigned int gen2Priority =0;
unsigned int gen3Priority =0;
unsigned int gen4Priority =0;

EthernetClient modules[NUMBER_OF_DSE];
ModbusTCPClient modbusTCPClient[NUMBER_OF_DSE]={
  ModbusTCPClient(modules[0]),
  ModbusTCPClient(modules[1])
};

IPAddress servers[NUMBER_OF_DSE]={//IP Addresses of the Servers
  IPAddress(192, 168, 137,  128),
  IPAddress(192, 168, 137,  126)
};

EthernetServer server(502);
ModbusTCPServer modbusTCPServer;
EthernetClient clients[NUMBER_OF_MODBUS_CLIENTS];//numero de clientes que pueden conectarse al servidor modbus local
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

  ///////////////////////
  //update date event
  updateDateEvent.repeat();
  updateDateEvent.start();

  ///////////////////////
  //reconnect
  dseReconnect.repeat();
  dseReconnect.start();

  //////////////////////
  //ETHERNET-MODBUS
  Ethernet.begin(mac, ip);
  modbusTCPClient[0].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[1].setTimeout(MODBUS_TIMEOUT);

  server.begin();
  if(!modbusTCPServer.begin()){
    Serial.println("Failed to start Modbus TCP server");
    while(1);
  }else{
    Serial.println("Modbus TCP server initialized");
  }
  modbusTCPServer.configureDiscreteInputs(0x00,NUMBER_OF_DSE*150);
  modbusTCPServer.configureCoils(0x00,100);
  modbusTCPServer.configureInputRegisters(0x00,NUMBER_OF_DSE*37);
  modbusTCPServer.configureHoldingRegisters(0x00,10);

  initializeArrays();

}//FIN SETUP

////////
//LOOP//
////////
void loop(){

  busLive = gen1Breaker || gen2Breaker || gen3Breaker || gen4Breaker || master1BusAvailable || master2BusAvailable || master3BusAvailable || master4BusAvailable;
  generalCommonAlarm = gen1CommonAlarm || gen2CommonAlarm || gen3CommonAlarm || gen4CommonAlarm || master1CommonAlarm || master2CommonAlarm || master3CommonAlarm || master4CommonAlarm;

  handleModbusClients();
  readDseAlarms();
  computeDseAlarms();
  test();


  if(dseReconnect.run()){
    for(int i=0; i<NUMBER_OF_DSE; i++){
      dseErrorComm[i]=false;
    }
  }

  if(updateDateEvent.run()){
    //activar coil de update date
  }

  updateInputs();
  updateCoils();
  updateInputRegisters();
  updateHoldingRegisters();

  /////////////////////////////
  //UTILIDADES
  utilidades();
}//FIN LOOP

////////////////////////////////////////////////
void test(){
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      dseIR[i][j]=12657;
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
            dseIR[d][i] = modbusTCPClient[d].read();
          }
        }
      }else{
        Serial.println(modbusTCPClient[d].lastError());
        dseErrorComm[d]=true;
      }
    }
  }
}

void computeDseAlarms(){
  //esta funcion separa las 4 alarmas que vienen dentro del mismo registro
  //cada alarma es de 4 bits y solo se busca que este entre los valores de 2 a 4
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){

      int a1 = dseIR[i][j]&0b00001111;
      int a2 = (dseIR[i][j]>>4)&0b00001111;
      int a3 = (dseIR[i][j]>>8)&0b00001111;
      int a4 = (dseIR[i][j]>>12)&0b00001111;

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

void updateCoils(){
  modbusTCPServer.coilWrite(0,updateModulesDates);
  modbusTCPServer.coilWrite(1,gensetPriorityError);

  modbusTCPServer.coilWrite(10,busLive);
  gen1Breaker = modbusTCPServer.coilRead(11);
  gen2Breaker = modbusTCPServer.coilRead(12);
  gen3Breaker = modbusTCPServer.coilRead(13);
  gen4Breaker = modbusTCPServer.coilRead(14);
  master1BusAvailable = modbusTCPServer.coilRead(15);
  master2BusAvailable = modbusTCPServer.coilRead(16);
  master3BusAvailable = modbusTCPServer.coilRead(17);
  master4BusAvailable = modbusTCPServer.coilRead(18);

  modbusTCPServer.coilWrite(20,generalCommonAlarm);
  gen1CommonAlarm = modbusTCPServer.coilRead(21);
  gen2CommonAlarm = modbusTCPServer.coilRead(22);
  gen3CommonAlarm = modbusTCPServer.coilRead(23);
  gen4CommonAlarm = modbusTCPServer.coilRead(24);
  master1CommonAlarm = modbusTCPServer.coilRead(25);
  master2CommonAlarm = modbusTCPServer.coilRead(26);
  master3CommonAlarm = modbusTCPServer.coilRead(27);
  master4CommonAlarm = modbusTCPServer.coilRead(28);
}

void updateInputRegisters() {//Read the Holding Register words from the Modbus Library
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      modbusTCPServer.inputRegisterWrite((i*37)+j,dseIR[i][j]);
    }
  }
}

void updateHoldingRegisters(){
  modbusTCPServer.holdingRegisterWrite(0,gen1Priority);
  modbusTCPServer.holdingRegisterWrite(1,gen2Priority);
  modbusTCPServer.holdingRegisterWrite(2,gen3Priority);
  modbusTCPServer.holdingRegisterWrite(3,gen4Priority);
}





void initializeArrays(){
    for(int i=0;i<NUMBER_OF_DSE;i++){
      for(int j=0;j<150;j++){
        dseAlarms[i][j]=0;
      }
    }
    for(int i=0;i<NUMBER_OF_DSE;i++){
      for(int j=0;j<37;j++){
        dseIR[i][j]=false;
      }
    }
    for(int i=0; i<NUMBER_OF_DSE; i++){
      dseErrorComm[i]=false;
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

void utilidades(){
  //updating frame
  frame = micros() - beforeFrame;
  beforeFrame = micros();
  //printing frame time and memory usage
  if(frameEvent.run()){
    Serial.print("Frame: ");
    Serial.println(frame);
    Serial.print(F("MEM FREE: "));
    Serial.println(freeMemory(), DEC);
    Serial.print("DSE Errors: ");
    for(int i=0; i<NUMBER_OF_DSE;i++){
      Serial.print(dseErrorComm[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
