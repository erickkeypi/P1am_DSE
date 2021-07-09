/*
  PROGRAMA PARA COMPLEMENTAR LA PANTALLA CMORE TRABAJANDO EN CONJUNTO CON ESTA
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//LIBRERIAS
// #include <SDU.h>
#include <P1AM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <SPI.h>
#include <Ethernet.h>
#include <RTCZero.h>
#include <MemoryFree.h>
#include <SD.h>
#include <KontrolMin.h>
#include <TimeEvent.h>
#include "DSEAlarms.h"
#include <DSE.h>
// #include <StateMachine.h>

//////////////////////////////////////////////////////
//MACROS
#define NUMBER_OF_DSE 8//SI SON MAS DE 8 MODULOS SE DEBE CONFIGURAR LOS OBJETOs DE EthernetClient y ModbusTCPClient
#define NUMBER_OF_MODBUS_CLIENTS 1  //CANTIDAD DE CLIENTES QUE PUEDEN CONETARSE POR MODBUS
#define MODBUS_RECONNECT_TIME 15000 //TIEMPO DE RECONEXION DE LOS MODULOS
#define UPDATE_DATE_PERIOD 1296000000 //LA FECHA DE LOS DSE SE ACTUALIZAN CADA 15 DIAS
#define RTC_UPDATE_TIME 300000 // se actualiza el rtc cada 5 minutos
#define READ_DSE_PERIOD 1000 //la lectura de los valores de los DSE es cada segundo
//MODOS DE LECTURA
#define READ_MASTER_AND_GEN 0
#define READ_ONLY_MASTER 1
#define READ_ONLY_GEN 2

//macros para tipo de repeticion del schedule ( diario, mensual, semanal, o en fecha especifica)
#define SCH_DAILY 0
#define SCH_WEEKLY 1
#define SCH_MONTHLY 2
#define SCH_DATE 3
//macros de modos para que funcione se debe hacer una logica en el DSE
#define SCH_TEST_OFF_LOAD false
#define SCH_TEST_ON_LOAD true
#define SCH_TRANSITION_OPEN false
#define SCH_TRANSITION_CLOSED true

//MODULO BASE PARA LA LECTURA DE LA FECHA
#define DSE_BASE_8660 0
#define DSE_BASE_8610 1
//
//////////////////////////////////////////////////////
//UTILIDADES
//UTILIZADAS PARA INFORMACION EXTRA
//NO SON NECESARIAS PARA EL FUNCIONAMIENTO DEL PROGRAMA
unsigned long frame=0;
unsigned long beforeFrame = 0;
TimeEvent frameEvent = TimeEvent(1000);
// bool debug = true;
bool debugUtilidades = false;
//
//////////////////////////////////////////////////////
//KONTROL
KontrolMin kontrol = KontrolMin();

//////////////////////////////////////////////////////
//RTC
RTCZero rtc;

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA ACTUALIZAR LA FECHA DE LOS DSE
TimeEvent updateDateEvent = TimeEvent(UPDATE_DATE_PERIOD);

unsigned long updateDateLastTime = 0;//variable complementaria utilizada para hacer que el boton de actualizar fecha deje de estar presionado
bool updatingDate = false;//variable complementaria para actualizar la fecha de los DSE
bool updateModulesDates = false;//VARIABLE QUE SE ACTIVA CUANDO SE VAYAN A ACTUALIZAR LOS MODULOS DSE
int dseBase = DSE_BASE_8660;//numero de modulo DSE de donde se leera la fecha

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA RECONECCION DE LOS DSE CUANDO SE PIERDE LA COMUNICACION
TimeEvent dseReconnect = TimeEvent(MODBUS_RECONNECT_TIME);

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA ACTUALIZAION DEL RTC
TimeEvent rtcUpdate = TimeEvent(10000);//Al principio el RTC trata de actualizarse cada 10 segundos luego pasa al tiempo definido por UPDATE_DATE_PERIOD

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA LECTURA DE LOS REGISTROS DE LOS DSE
TimeEvent readDseTimer = TimeEvent(READ_DSE_PERIOD);

//////////////////////////////////////////////////////
//ARRAYS
bool dseErrorComm[NUMBER_OF_DSE];//error de comunicacion
bool oldDseErrorComm[NUMBER_OF_DSE];//ESTADO ANTERIOR DE LOS ERRORES
bool oldDseAlarms[NUMBER_OF_DSE][150];//ESTADO ANTERIOR DE LAS ALARMAS

// //////////////////////////////////////////////////////
// //PANTALLAS
unsigned int masterScreen[60];//VARIABLES PARA LA PANTALLA DE MASTER
unsigned int masterActual = 0;//MASTER DEL CUAL SE ESTA LEYENDO LOS VALORES
bool masterButtonPress = false;//ESTADO DE BOTON DE COMANDO DE MASTER
//
unsigned int genScreen[60];//VARIABLES PARA LA PANTALLA DE GEN
unsigned int genActual = 1;//GEN DEL CUAL SE ESTA LEYENDO LOS VALORES
bool genButtonPress = false;//ESTADO DE BOTON DE COMANDO DE GEN

bool muteAllAlarms = false;
//////////////////////////////////////////////////////
//MODOS DE LECTURA PARA HACER QUE SOLO LEA MASTERS O GENERADORES
int modoLectura = READ_MASTER_AND_GEN;

//////////////////////////////////////////////////////
//VARIABLES PARA DETERMINAR SI EL BUS ESTA CALIENTE
bool busLive = false;

//////////////////////////////////////////////////////
//VARIABLES PARA DETERMINAR LA ALARMA COMUN GENERAL
bool generalCommonAlarm = false;

//////////////////////////////////////////////////////
//CONFIGURACION ETHERNET-MODBUS
byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0x68, 0x98};//P1AM-ETH MAC
IPAddress ip(10, 0, 0, 19);//P1AM-ETH IP

//////////////////////////////////////////////////////
//CONFIGURACION DEL SERVIDOR MODBUS
EthernetServer server(502);
ModbusTCPServer modbusTCPServer;
EthernetClient clients[NUMBER_OF_MODBUS_CLIENTS];
int client_cnt=0;//VARIABLE QUE GUARDA EL NUMERO DE CLIENTES CONECTADOS

//////////////////////////////////////////////////////
//SCHEDULE
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

//////////////////////////////////////////////////////
//SD CARD
const int chipSelect = SDCARD_SS_PIN;
String dataWriteSD;

//////////////////////////////////////////////////////
//VARIABLES PANTALLA DE ALARMA Y EVENT LOG
char alarmLine[60];//ARRAY DE TEXTO DE ALARMAS
unsigned int tabla = 0;//TABLA DE EVENTLOG QUE SE ESTA LEYENDO
unsigned int tablaActive = 0 ;//TABLA DE ALARMA ACTIVA QUE SE ESTA LEYENDO
unsigned int monthTable = 1;//MES DE LECTURA
unsigned int yearTable = 20;//AÑO DE LECTURA

//////////////////////////////////////////////////////
//MODULOS DSE
DSE modulos[NUMBER_OF_DSE] = {
  DSE(DSE_8660MKII, IPAddress(10, 0, 0,  11), "SBA1"),//11
  DSE(DSE_8610MKII, IPAddress(10, 0, 0,  15), "Gen 1"),//15
  DSE(DSE_8660MKII, IPAddress(10, 0, 0,  12), "SBA2"),
  DSE(DSE_8660MKII, IPAddress(10, 0, 0,  13), "SBB2"),
  DSE(DSE_8660MKII, IPAddress(10, 0, 0,  14), "SBB1"),
  DSE(DSE_8610MKII, IPAddress(10, 0, 0,  16), "Gen 2"),
  DSE(DSE_8610MKII, IPAddress(10, 0, 0,  17), "Gen 3"),
  DSE(DSE_8610MKII, IPAddress(10, 0, 0,  18), "Gen 4")
};



bool changePriority = 0;
bool oldPriority = 0;
unsigned int newPriority1 =1;
unsigned int newPriority2 =1;
unsigned int newPriority3 =1;
unsigned int newPriority4 =1;
unsigned int newPriority[4] = {1,1,1,1};

unsigned long totalMainsKW = 0;

unsigned long periodoLed = 1000;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////SETUP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  pinMode(LED_BUILTIN,OUTPUT);//LED FRONTAL
  pinMode(SWITCH_BUILTIN,INPUT);//SWITCH FRONTAL
  Serial.begin(115200);//COMUNICACION SERIAL
  digitalWrite(LED_BUILTIN,HIGH);
  //////////////////////////////////////////////////////
  //UTILIDADES
  frameEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  frameEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //UTILIDADES
  dataWriteSD.reserve(30);//RESERVANDO ESPACIO PARA EL STRING
  dataWriteSD = F("123456789012345678901234567890");//INICIANDO EL STRING CON VALORES ALEATORIOS

  asignAlarm();//ASINGANDO EL TEXTO DE LAS ALARMAS AL ARRAY DSEAlarmsString. FUNCION Y ARRAY DE DSEAlarms.h

  delay(2000);//RETARDO PARA EL INICIO DEL PROGRAMA

  Serial.println(F("\n**********INIT**********"));//MARCANDO EL INICIO DEL PROGRAMA
  // Serial.println(F("**< SDU compatible >**"));
  if(digitalRead(SWITCH_BUILTIN)){
    SD_Begin();//INICIANDO MICRO SD
  }
  Serial.println(F("> Iniciando RTC"));
  rtc.begin();///INICIANDO RTC

  //////////////////////////////////////////////////////
  //COMFIGURANDO TIMER DE ACTUALIZACION DE FECHAS
  updateDateEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  updateDateEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //COMFIGURANDO TIMER DE LECTURA DE LOS MODULOS
  readDseTimer.repeat();
  readDseTimer.start();

  //////////////////////////////////////////////////////
  //CONFIGURANDO TIMER DE RECONECCION
  dseReconnect.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  // dseReconnect.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //CONFIGURANDO TIMER DE RTC
  rtcUpdate.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  rtcUpdate.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////
  //ETHERNET-MODBUS
  Ethernet.begin(mac, ip);

  //////////////////////////////////////////////////////
  //INICIANDO SERVIDOR MODBUS
  Serial.println(F("> Iniciando servidor Modbus"));
  server.begin();
  if(!modbusTCPServer.begin()){//SI NO SE INICIA EL SERVIDOR MODBUS SE BLOQUEA EL PLC
    Serial.println(F("> Fallo al iniciar servidor Modbus"));
    //ACTIVAR LED DE ERROR
    while(1);
  }else{//SI SE INICIA EL SERVIDOR MODBUS
    Serial.print(F("> Servidor Modbus iniciado, IP: "));
    Serial.println(ip);
  }
  //////////////////////////////////////////////////////
  //CONFIGURANDO REGISTROS MODBUS
  modbusTCPServer.configureDiscreteInputs(0X00,200);
  modbusTCPServer.configureCoils(0x00,200);
  modbusTCPServer.configureInputRegisters(0x00,10);
  modbusTCPServer.configureHoldingRegisters(0x00,1600);
  Serial.println(F("> Registros Modbus configurados"));

  initializeArrays();//inicializando los arrays

  //////////////////////////////////////////////////////
  //INICIALIZANDO MODULOS
  for(int i=0; i<NUMBER_OF_DSE;i++){
    modulos[i].begin();
  }
  readDse();//PRIMERA LECTURA DE LOS MODULOS
  readModuleDate();//LEYENDO Y ACTUALIZANDO RTC DEL PLC TOMANDO COMO REFERENCIA RTC DE MODULO BASE

  //////////////////////////////////////////////////////
  //INICIALIZANDO DATALOGGER
  if(digitalRead(SWITCH_BUILTIN)){
    SD.remove(F("ACTIVE.csv"));//BORRANDO LAS ALARMAS ACTIVAS
    dataloggerInit();//INICIANDO EL DATALOGGER
    dataWriteSD = F("PLC REINICIADO");
    datalogger();
    dataloggerRead(rtc.getMonth(),rtc.getYear());//LEYENDO EL DATALOGGER DEL MES Y AÑO ACTUAL
  }
  modbusTCPServer.holdingRegisterWrite(1240,rtc.getMonth());
  modbusTCPServer.holdingRegisterWrite(1241,rtc.getYear());
  Serial.println(F("> Setup finalizado"));
}//FIN SETUP

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////LOOP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  static unsigned long tiempoLed = millis();

  if(digitalRead(SWITCH_BUILTIN)){
    periodoLed = 1000;
  }else{
    periodoLed = 200;
  }

  if(millis() - tiempoLed > periodoLed){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));//PAPADEO DEL LED FRONTAL
    tiempoLed = millis();
  }

  //LOGICA PARA BUS LIVE Y LA ALARMA COMUN GENERAL
  busLive = generalCommonAlarm = false;
  for (int i=0;i<NUMBER_OF_DSE;i++){
    if(modulos[i].model == DSE_8660MKII){
      busLive |= modulos[i].busLive;
    }
    else if(modulos[i].model == DSE_8610MKII){
      busLive |= modulos[i].busLive;
    }
    generalCommonAlarm |= modulos[i].commonAlarm;
  }

  //////////////////////////////////////////////////////
  //TIMER DE LECTURA DE LOS REGISTROS DE LOS DSE
  if(readDseTimer.run()){
    //////////////////////////////////////////////////////
    //LOGICA DE RECONEXION
    bool errorComm = false;
    for (int i=0;i<NUMBER_OF_DSE;i++){//determinando si hay algun error de conexion
      if(dseErrorComm[i]){
        errorComm = true;
        break;
      }
    }

    if(errorComm){// si hay un error de conexion se empieza a contar el tiempo de reconexion
      dseReconnect.start();
    } else {// si no hay error de conexion se detiene la cuenta
      dseReconnect.stop();
    }

    if(dseReconnect.run()){//timer de reconexion
      Serial.println(F("> Reconexion"));
      eraseErrorComm();//QUITANDO LOS ERRORES DE CONEXION PARA PERMITIR LA RECONEXION
    }

    applyReadMode();//APLICANDO MODO DE LECTURA SEGUN LA VARIABLE "modoLectura"
    readDse();//LEYENDO LAS LOS REGISTROS DE LOS DSE
    if(digitalRead(SWITCH_BUILTIN)){
      activateAlarms();//REVISANDO LAS ALARMAS PARA AGREGARLAS AL DATALOGGER
    }
  }

  //////////////////////////////////////////////////////
  //LOGICA PARA EL MANEJO DE LA PANTALLA DE EVENT LOG
  unsigned int tablaServer = modbusTCPServer.holdingRegisterRead(1243);//TABLA QUE SE QUIERE MOSTRAR
  unsigned int monthServer = modbusTCPServer.holdingRegisterRead(1240);//MES QUE SE QUIERE MOSTRAR
  unsigned int yearServer = modbusTCPServer.holdingRegisterRead(1241);//AÑO QUE SE QUIERE MOSTRAR

  if(tablaServer != tabla || monthTable != monthServer || yearTable != yearServer){//SI CAMBIA EL MES, EL AÑO O LA TABLA QUE SE ESTA MOSTRANDO
    if(tablaServer>10000){//LIMITANDO EL NUMERO DE TABLA
      tablaServer = 0;
    }
    //CAMBIANDO A LOS NUEVOS VALORES QUE SE QUIEREN MOSTRAR
    monthTable = monthServer;
    yearTable = yearServer;
    tabla = tablaServer;
    modbusTCPServer.holdingRegisterWrite(1243,tabla);//ESCRIBIENDO EN EL REGISTRO EL NUEVO NUMERO DE TABLA QUE SE ESTA MOSTRANDO
    if(digitalRead(SWITCH_BUILTIN)){
      dataloggerRead(monthTable,yearTable);//MOSTRANDO EL EVENTLOG
    }
  }

  //////////////////////////////////////////////////////
  //LOGICA PARA EL MANEJO DE LA PANTALLA DE ALARMAS ACTIVAS
  tablaServer = modbusTCPServer.holdingRegisterRead(1563);//TABLA QUE SE QUIERE MOSTRAR
  if(tablaServer != tablaActive){//SI CAMBIA LA TABLA QUE SE ESTA MOSTRANDO
    if(tablaServer>10000){//LIMITANDO
      tablaServer = 0;
    }
    tablaActive = tablaServer;
    modbusTCPServer.holdingRegisterWrite(1563,tablaActive);//ESCRIBIENDO EN EL REGISTRO EL NUEVO NUMERO DE TABLA QUE SE ESTA MOSTRANDO
    if(digitalRead(SWITCH_BUILTIN)){
      alarmsLoggerRead();//MOSTRANDO LAS ALARMAS ACTIVAS
    }
  }

  //////////////////////////////////////////////////////
  //MANEJANDO LOS CLIENTES
  handleModbusClients();

  //////////////////////////////////////////////////////
  //KONTROL
  if(Serial.available()>0){
    kontrol.update(Serial.read());
  }
  kontrol.addListener("ok",okCallback);
  kontrol.addListener("help",helpCall);
  kontrol.addListener("updatedate",updateDateCallback);
  kontrol.addListener("debug",debugCallback);
  kontrol.addListener("connectmodules",eraseErrorComm);
  kontrol.addListener("printmemory",printMemory);
  kontrol.addListener("readmode",modoLecturaCallback);
  kontrol.addListener("test",testCallback);
  kontrol.addListener("test2",test2Callback);
  kontrol.addListener("test3",test3Callback);

  //////////////////////////////////////////////////////
  //ACTUALIZANDO LA FECHA DE LOS MODULOS Y DEL PLC
  if(updateDateEvent.run()){//TIMER DE ACTUALIZACION DE FECHA DE LOS MODULOS
    updateModulesDates = true;
  }
  if(updateModulesDates && !updatingDate){
    Serial.println(F("> Actualizando fechas de los modulos y PLC"));
    readModuleDate();
    updatingDate = true;
    updateDateLastTime = millis();
  }
  if(updatingDate && (millis() > (updateDateLastTime + 1000))){
    updateModulesDates = false;
    updatingDate =false;
    updateDseDates();
    Serial.println(F("> Fecha de modulos actualizadas"));
  }

  //////////////////////////////////////////////////////
  //TIMER DE ACTUALIZACION DE RTC
  if(rtcUpdate.run()){
    readModuleDate();
  }

  //////////////////////////////////////////////////////
  //LOGICA DEL SCHEDULE
  computeSchRegisters();
  if(schDurationTimer.run()){
    schActive = false;
    schDurationTimer.stop();
    Serial.println(F("> Schedule desactivado"));
  }

  //////////////////////////////////////////////////////
  //ACTUALIZANDO LOS REGISTROS MODBUS
  writeModbusCoils();
  writeModbusDiscreteInputs();
  writeModbusInputRegisters();
  writeModbusHoldingRegisters();

  /////////////////////////////
  //UTILIDADES
  utilidades();
}//FIN LOOP
