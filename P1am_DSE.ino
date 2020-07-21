/*
  PROGRAMA PARA COMPLEMENTAR LA PANTALLA CMORE TRABAJANDO EN CONJUNTO CON ESTA

  EN ESTE PROGRAMA SE LEEN Y SEPARAN LAS ALARMAS DE LOS MODULOS DSE
  ADEMAS TAMBIEN SE HACEN OPERACIONES COMPLEMENTARIAS

  LAS ALARMAS SE GUARDAN EN LAS ENTRADAS DISCRETAS DE LOS REGISTROS MODBUS
  TIPO DE MEMORIA MODBUS 1 DESDE EL REGISTRO 0 HASTA EL REGISTRO 150 * NUMBER_OF_DSE
  A CADA DSE LE CORRESPONDE 150 REGISTROS PARA GUARDAR SUS ALARMAS

  LA CMORE UTILIZA LOS COILS PARA FUNCIONES COMPLEMENTARIAS (TIPO DE MEMORIA MODBUS 0)
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//HACER LA LOGICA PARA ACTUALIZAR LA FECHA DE LOS DSE
//HACER LOGICA PARA LIMPIAR LAS ALARMAS DE UN MODULO QUE SE HAYA DESCONECTADO
//HACER SCHEDULE
//HACER LOG DE ERRORES EN SD
//HACER LED DE ERROR Y LED DE WARNING
//HACER UN ESTADO EN EL QUE ESTA ACTIVO UN ERROR Y NO EJECUTAR MODBUS
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//LIBRERIAS
#include <P1AM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <TimeEvent.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MemoryFree.h>;

//////////////////////////////////////////////////////
//MACROS
#define NUMBER_OF_DSE 2//SI SON MAS DE 8 MODULOS SE DEBE CONFIGURAR LOS OBJETOS DE EthernetClient y ModbusTCPClient de la lineas 95 y 96
#define NUMBER_OF_MODBUS_CLIENTS 8  //CANTIDAD DE CLIENTES QUE PUEDEN CONETARSE POR MODBUS
#define MODBUS_TIMEOUT 200
#define MODBUS_RECONNECT_TIME 30000
#define UPDATE_DATE_PERIOD 1296000000 //LA FECHA DE LOS DSE SE ACTUALIZAN CADA 15 DIAS

//////////////////////////////////////////////////////
//UTILIDADES
//UTILIZADAS PARA INFORMACION EXTRA
//NO SON NECESARIAS PARA EL FUNCIONAMIENTO DEL PROGRAMA
unsigned long frame=0;
unsigned long beforeFrame = 0;
TimeEvent frameEvent = TimeEvent(1000);
bool debug = true;
bool debugUtilidades = false;

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA ACTUALIZAR LA FECHA DE LOS DSE
TimeEvent updateDateEvent = TimeEvent(UPDATE_DATE_PERIOD);

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA RECONECCION DE LOS DSE CUANDO SE PIERDE LA COMUNICACION

TimeEvent dseReconnect = TimeEvent(MODBUS_RECONNECT_TIME);

//////////////////////////////////////////////////////
//ARRAYS
int dseIR[NUMBER_OF_DSE][37];//alarmas leidas
bool dseAlarms[NUMBER_OF_DSE][150];//bits de las alarmas
bool dseErrorComm[NUMBER_OF_DSE];//error de comunicacion

bool updateModulesDates = false;//VARIABLE QUE SE ACTIVA CUANDO SE VAYAN A ACTUALIZAR LOS MODULOS DSE

//////////////////////////////////////////////////////
//VARIABLES PARA DETERMINAR SI EL BUS ESTA CALIENTE
bool busLive = false;
bool gen1Breaker = false;
bool gen2Breaker = false;
bool gen3Breaker = false;
bool gen4Breaker = false;
bool master1BusAvailable = false;
bool master2BusAvailable = false;
bool master3BusAvailable = false;
bool master4BusAvailable = false;

//////////////////////////////////////////////////////
//VARIABLES PARA DETERMINAR LA ALARMA COMUN GENERAL
bool generalCommonAlarm = false;
bool gen1CommonAlarm =false;
bool gen2CommonAlarm =false;
bool gen3CommonAlarm =false;
bool gen4CommonAlarm =false;
bool master1CommonAlarm = false;
bool master2CommonAlarm = false;
bool master3CommonAlarm = false;
bool master4CommonAlarm = false;

//////////////////////////////////////////////////////
//CONFIGURACION ETHERNET-MODBUS
byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0x68, 0x98};//P1AM-ETH MAC
IPAddress ip(192, 168, 137, 177);//P1AM-ETH IP
EthernetClient modules[8];//CANTIDAD MAXIMA DE MODULOS ES DE 8
ModbusTCPClient modbusTCPClient[8]={
  ModbusTCPClient(modules[0]),
  ModbusTCPClient(modules[1]),
  ModbusTCPClient(modules[2]),
  ModbusTCPClient(modules[3]),
  ModbusTCPClient(modules[4]),
  ModbusTCPClient(modules[5]),
  ModbusTCPClient(modules[6]),
  ModbusTCPClient(modules[7])
};
//IPs DE LOS MODULOS DSE
//AGREGAR TANTAS IPs COMO MODULOS DSE
IPAddress servers[NUMBER_OF_DSE]={
  IPAddress(192, 168, 137,  128),
  IPAddress(192, 168, 137,  126)
};

//////////////////////////////////////////////////////
//CONFIGURACION DEL SERVIDOR MODBUS
EthernetServer server(502);
ModbusTCPServer modbusTCPServer;
EthernetClient clients[NUMBER_OF_MODBUS_CLIENTS];
int client_cnt=0;//VARIABLE QUE GUARDA EL NUMERO DE CLIENTES CONECTADOS

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////SETUP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //////////////////////////////////////////////////////
  //UTILIDADES
  Serial.begin(115200);
  delay(2000);
  //while(!Serial){}
  if(debug){
    Serial.println("INIT");
  }
  frameEvent.repeat();
  frameEvent.start();

  //////////////////////////////////////////////////////
  //COMFIGURANDO TIMER DE ACTUALIZACION DE FECHAS
  updateDateEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  updateDateEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //CONFIGURANDO TIMER DE RECONECCION
  dseReconnect.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  dseReconnect.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////
  //ETHERNET-MODBUS
  Ethernet.begin(mac, ip);
  modbusTCPClient[0].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[1].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[2].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[3].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[4].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[5].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[6].setTimeout(MODBUS_TIMEOUT);
  modbusTCPClient[7].setTimeout(MODBUS_TIMEOUT);

  //////////////////////////////////////////////////////
  //INICIANDO SERVIDOR MODBUS
  server.begin();
  if(!modbusTCPServer.begin() && debug){
    Serial.println("Failed to start Modbus TCP server");
    //ACTIVAR LED DE ERROR
    while(1);
  }else{
    if(debug){
      Serial.println("Modbus TCP server initialized");
    }
  }
  //////////////////////////////////////////////////////
  //CONFIGURANDO REGISTROS MODBUS
  modbusTCPServer.configureDiscreteInputs(0x00,NUMBER_OF_DSE*150);
  modbusTCPServer.configureCoils(0x00,100);
  modbusTCPServer.configureInputRegisters(0x00,NUMBER_OF_DSE*37);
  modbusTCPServer.configureHoldingRegisters(0x00,10);

  initializeArrays();//inicializando los arrays

}//FIN SETUP

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////LOOP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //LOGICA PARA BUS LIVE Y LA ALARMA COMUN GENERAL
  busLive = gen1Breaker || gen2Breaker || gen3Breaker || gen4Breaker || master1BusAvailable || master2BusAvailable || master3BusAvailable || master4BusAvailable;
  generalCommonAlarm = gen1CommonAlarm || gen2CommonAlarm || gen3CommonAlarm || gen4CommonAlarm || master1CommonAlarm || master2CommonAlarm || master3CommonAlarm || master4CommonAlarm;

  handleModbusClients();//MANEJANDO LOS CLIENTES
  readDseAlarms();//LEYENDO LAS LOS REGISTROS DE ALARMAS DE LOS DSE
  computeDseAlarms();//SEPARANDO LAS ALARMAS QUE VIENEN EN EL MISMO REGISTRO

  test();//FUNCION PARA REALIZAR PRUEBAS

  //TIMER DE RECONEXION
  if(dseReconnect.run()){
    for(int i=0; i<NUMBER_OF_DSE; i++){
      //SE DESACTIVA EL ERROR DE CONEXION PARA QUE LA FUNCION QUE LEE LOS MODULOS INTENTE CONECTARSE
      dseErrorComm[i]=false;
    }
  }

  //TIMER DE ACTUALIZACION DE FECHA DE LOS MODULOS
  if(updateDateEvent.run()){
    //activar coil de update date
  }

  //ACTUALIZANDO LOS REGISTROS MODBUS
  updateDiscreteInputs();
  updateCoils();
  updateInputRegisters();
  updateHoldingRegisters();

  /////////////////////////////
  //UTILIDADES
  utilidades();
}//FIN LOOP
