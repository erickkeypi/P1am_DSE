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
//HACER LOG DE ERRORES EN SD
//HACER LED DE ERROR Y LED DE WARNING
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
#include <RTCZero.h>
#include <MemoryFree.h>
#include <KontrolMin.h>
#include <StateMachine.h>

//////////////////////////////////////////////////////
//MACROS
#define NUMBER_OF_DSE 7//SI SON MAS DE 8 MODULOS SE DEBE CONFIGURAR LOS OBJETOs DE EthernetClient y ModbusTCPClient
#define NUMBER_OF_MODBUS_CLIENTS 1  //CANTIDAD DE CLIENTES QUE PUEDEN CONETARSE POR MODBUS
#define MODBUS_TIMEOUT 200
#define MODBUS_RECONNECT_TIME 60000
#define UPDATE_DATE_PERIOD 1296000000 //LA FECHA DE LOS DSE SE ACTUALIZAN CADA 15 DIAS
#define RTC_UPDATE_TIME 300000 // se actualiza el rtc cada 5 minutos

//////////////////////////////////////////////////////
//UTILIDADES
//UTILIZADAS PARA INFORMACION EXTRA
//NO SON NECESARIAS PARA EL FUNCIONAMIENTO DEL PROGRAMA
unsigned long frame=0;
unsigned long beforeFrame = 0;
TimeEvent frameEvent = TimeEvent(1000);
bool debug = true;
bool debugUtilidades = true;

KontrolMin kontrol = KontrolMin();//KONTROL
RTCZero rtc;//RTC

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA ACTUALIZAR LA FECHA DE LOS DSE
TimeEvent updateDateEvent = TimeEvent(UPDATE_DATE_PERIOD);
unsigned long updateDateLastTime = 0;//variable complementaria utilizada para hacer que el boton deje de estar presionado
bool updatingDate = false;//variable complementaria para actualizar la fecha de los DSE
bool updateModulesDates = false;//VARIABLE QUE SE ACTIVA CUANDO SE VAYAN A ACTUALIZAR LOS MODULOS DSE

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA RECONECCION DE LOS DSE CUANDO SE PIERDE LA COMUNICACION
TimeEvent dseReconnect = TimeEvent(MODBUS_RECONNECT_TIME);

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA ACTUALIZAION DEL RTC
TimeEvent rtcUpdate = TimeEvent(10000);//Al principio el RTC trata de actualizarse cada 10 segundos luego pasa al tiempo definido por UPDATE_DATE_PERIOD

//////////////////////////////////////////////////////
//ARRAYS
int dseIR[NUMBER_OF_DSE][37];//alarmas leidas
bool dseAlarms[NUMBER_OF_DSE][150];//bits de las alarmas
bool dseErrorComm[NUMBER_OF_DSE];//error de comunicacion
unsigned int variablesPrincipales[NUMBER_OF_DSE][20];//array para guardar valores que se presentan en la pantalla principal
char nombres[7][12] = {//LOS NOMBRES NO PUEDEN TENER MAS DE 11 CARACTERES
  "Master SBA1",
  "Gen 1",
  "Master SBA2",
  "Master SBB1",
  "Master SBB2",
  "Gen 2",
  "Gen 3"
};

bool dseInputs[8][10];
unsigned int masterScreen[60];
unsigned int masterActual = 0;
bool masterButtonPress = false;
unsigned int genActual = 1;
unsigned int genScreen[60];
bool genButtonPress = false;

//
// bool masterXMainAvailable = false;
// bool masterXBusAvailable = false;
// bool masterXLoadOn = false;
// bool masterXMainBrk = false;
// bool masterXBusBrk = false;
// unsigned int masterXLLAVR = 0;



//////////////////////////////////////////////////////
//VARIABLES PARA DETERMINAR SI EL BUS ESTA CALIENTE
bool busLive = false;

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
EthernetClient modules[7];//CANTIDAD MAXIMA DE MODULOS ES DE 8
ModbusTCPClient modbusTCPClient[7]={
  ModbusTCPClient(modules[0]),
  ModbusTCPClient(modules[1]),
  ModbusTCPClient(modules[2]),
  ModbusTCPClient(modules[3]),
  ModbusTCPClient(modules[4]),
  ModbusTCPClient(modules[5]),
  ModbusTCPClient(modules[6])
};
//IPs DE LOS MODULOS DSE
//AGREGAR TANTAS IPs COMO MODULOS DSE
IPAddress servers[7]={
  IPAddress(192, 168, 137,  126),//MASTER1
  IPAddress(192, 168, 137,  128),//GEN1
  IPAddress(192, 168, 137,  127),//MASTER2
  IPAddress(192, 168, 137,  127),//MASTER3
  IPAddress(192, 168, 137,  126),//MASTER4
  IPAddress(192, 168, 137,  127),//GEN2
  IPAddress(192, 168, 137,  128)//GEN3
};

//////////////////////////////////////////////////////
//CONFIGURACION DEL SERVIDOR MODBUS
EthernetServer server(502);
ModbusTCPServer modbusTCPServer;
EthernetClient clients[NUMBER_OF_MODBUS_CLIENTS];
int client_cnt=0;//VARIABLE QUE GUARDA EL NUMERO DE CLIENTES CONECTADOS

//////////////////////////////////////////////////////
//SCHEDULE
//macros para tipo de repeticion del schedule ( diario, mensual, semanal, o en fecha especifica)
#define SCH_DAILY 0
#define SCH_WEEKLY 1
#define SCH_MONTHLY 2
#define SCH_DATE 3
//macros de modos
#define SCH_TEST_OFF_LOAD false
#define SCH_TEST_ON_LOAD true
#define SCH_TRANSITION_OPEN false
#define SCH_TRANSITION_CLOSED true

byte schTipoRepeticion = SCH_DATE;
bool schEnable = false;//activa el schedule
bool schActive = false;//se activa cuando se llega a la fecha y hora establecido en el sch

bool schTestLoad = SCH_TEST_OFF_LOAD;
bool schTransition = SCH_TRANSITION_OPEN;
bool schLoadDemandInhibit = false;

unsigned int schHour = 0;
unsigned int schMinute = 0;
unsigned int schDay = 1;
unsigned int schMonth = 1;
unsigned int schDuration = 1;

bool schCoils[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int schHolding[5]={0,0,0,0,0};

TimeEvent schDurationTimer = TimeEvent(5000);


unsigned int testInt = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////SETUP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //////////////////////////////////////////////////////
  //UTILIDADES
  pinMode(LED_BUILTIN,OUTPUT);//LED FRONTAL
  pinMode(SWITCH_BUILTIN,INPUT);//SWITCH FRONTAL
  Serial.begin(115200);//COMUNICACION SERIAL
  delay(2000);//RETARDO PARA EL INICIO DEL PROGRAMA
  frameEvent.repeat();
  frameEvent.start();

  //while(!Serial){}
  if(debug){
    Serial.println(F("\n**********INIT**********"));
  }

  Serial.println(F("> Iniciando RTC"));
  rtc.begin();

  //////////////////////////////////////////////////////
  //COMFIGURANDO TIMER DE ACTUALIZACION DE FECHAS
  updateDateEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  updateDateEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //CONFIGURANDO TIMER DE RECONECCION
  dseReconnect.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  dseReconnect.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //CONFIGURANDO TIMER DE RTC
  rtcUpdate.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  rtcUpdate.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

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
  Serial.println(F("> Iniciando servidor Modbus"));
  server.begin();
  if(!modbusTCPServer.begin() && debug){
    Serial.println(F("> Fallo al iniciar servidor Modbus"));
    //ACTIVAR LED DE ERROR
    while(1);
  }else{
    if(debug){
      Serial.print(F("> Servidor Modbus iniciado, IP: "));
      Serial.println(ip);
    }
  }
  //////////////////////////////////////////////////////
  //CONFIGURANDO REGISTROS MODBUS
  modbusTCPServer.configureDiscreteInputs(0X00,200);
  modbusTCPServer.configureCoils(0x00,200);
  modbusTCPServer.configureInputRegisters(0x00,NUMBER_OF_DSE*37);
  modbusTCPServer.configureHoldingRegisters(0x00,1000);
  Serial.println(F("> Registros Modbus configurados"));

  initializeArrays();//inicializando los arrays

  connectModules();
  readModuleDate();

}//FIN SETUP

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////LOOP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //LOGICA PARA BUS LIVE Y LA ALARMA COMUN GENERAL
  //para los master se toma dseInputs[d][1] y para los gen dseInputs[d][2]
  busLive = dseInputs[0][1] || dseInputs[1][2] || dseInputs[2][1] || dseInputs[3][1] || dseInputs[4][1] || dseInputs[5][2] || dseInputs[6][2] || dseInputs[7][2];
  generalCommonAlarm = gen1CommonAlarm || gen2CommonAlarm || gen3CommonAlarm || gen4CommonAlarm || master1CommonAlarm || master2CommonAlarm || master3CommonAlarm || master4CommonAlarm;

  handleModbusClients();//MANEJANDO LOS CLIENTES
  readDse();//LEYENDO LAS LOS REGISTROS DE ALARMAS DE LOS DSE
  computeDseAlarms();//SEPARANDO LAS ALARMAS QUE VIENEN EN EL MISMO REGISTRO

  //KONTROL
  if(Serial.available()>0){
    kontrol.update(Serial.read());
  }
  kontrol.addListener(F("ok"),okCallback);
  kontrol.addListener(F("help"),helpCall);
  kontrol.addListener(F("updateDate"),updateDateCallback);
  kontrol.addListener(F("debug"),debugCallback);
  kontrol.addListener(F("connectModules"),connectModules);
  kontrol.addListener(F("printMemory"),printMemory);

  //TIMER DE RECONEXION
  if(dseReconnect.run()){
    for(int i=0; i<NUMBER_OF_DSE; i++){
      //SE DESACTIVA EL ERROR DE CONEXION PARA QUE LA FUNCION QUE LEE LOS MODULOS INTENTE CONECTARSE
      dseErrorComm[i]=false;
    }
  }

  //TIMER DE ACTUALIZACION DE FECHA DE LOS MODULOS
  if(updateDateEvent.run()){
    updateModulesDates = true;
  }
  if(updateModulesDates && !updatingDate){
    Serial.println(F("> Actualizando fechas de los modulos"));
    readModuleDate();
    updatingDate = true;
    updateDateLastTime = millis();
  }

  if(updatingDate && (millis() > (updateDateLastTime + 1000))){
    updateModulesDates = false;
    updatingDate =false;
    Serial.println(F("> Fecha de modulos actualizadas"));
  }

  if(rtcUpdate.run()){
    readModuleDate();
  }

  computeSchRegisters();
  if(schDurationTimer.run()){
    schActive = false;
    schDurationTimer.stop();
    Serial.println(F("> Schedule desactivado"));
  }

  //ACTUALIZANDO LOS REGISTROS MODBUS
  writeModbusCoils();
  writeModbusDiscreteInputs();
  writeModbusInputRegisters();
  writeModbusHoldingRegisters();



  /////////////////////////////
  //UTILIDADES
  utilidades();

}//FIN LOOP
