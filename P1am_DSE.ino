/*
  PROGRAMA PARA COMPLEMENTAR LA PANTALLA CMORE TRABAJANDO EN CONJUNTO CON ESTA
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
#include <SD.h>
#include "DSEAlarms.h"

//////////////////////////////////////////////////////
//MACROS
#define NUMBER_OF_DSE 7//SI SON MAS DE 8 MODULOS SE DEBE CONFIGURAR LOS OBJETOs DE EthernetClient y ModbusTCPClient
#define NUMBER_OF_MODBUS_CLIENTS 1  //CANTIDAD DE CLIENTES QUE PUEDEN CONETARSE POR MODBUS
#define MODBUS_TIMEOUT 200
#define MODBUS_RECONNECT_TIME 30000
#define UPDATE_DATE_PERIOD 1296000000 //LA FECHA DE LOS DSE SE ACTUALIZAN CADA 15 DIAS
#define RTC_UPDATE_TIME 300000 // se actualiza el rtc cada 5 minutos

//MODOS DE LECTURA
#define READ_MASTER_AND_GEN 0
#define READ_ONLY_MASTER 1
#define READ_ONLY_GEN 2

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
int dseBase =0;//numero de modulo DSE de donde se leera la fecha

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA RECONECCION DE LOS DSE CUANDO SE PIERDE LA COMUNICACION
TimeEvent dseReconnect = TimeEvent(MODBUS_RECONNECT_TIME);

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA ACTUALIZAION DEL RTC
TimeEvent rtcUpdate = TimeEvent(10000);//Al principio el RTC trata de actualizarse cada 10 segundos luego pasa al tiempo definido por UPDATE_DATE_PERIOD

//////////////////////////////////////////////////////
//TIMER UTILIZADO PARA LA LECTURA DE LOS REGISTROS DE LOS DSE
TimeEvent readDseTimer = TimeEvent(1000);

//////////////////////////////////////////////////////
//ARRAYS
int dseIR[NUMBER_OF_DSE][37];//alarmas leidas
bool dseAlarms[NUMBER_OF_DSE][150];//bits de las alarmas
bool dseErrorComm[8];//error de comunicacion
bool oldDseErrorComm[8];//ESTADO ANTERIOR DE LOS ERRORES
bool oldDseAlarms[NUMBER_OF_DSE][150];//ESTADO ANTERIOR DE LAS ALARMAS
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

//////////////////////////////////////////////////////
//PANTALLAS
bool dseInputs[8][10];//GUARDA EL ESTADO DE LOS BREAKERS, LA DISPONIBILIDAD, ETC.
unsigned int masterScreen[60];//VARIABLES PARA LA PANTALLA DE MASTER
unsigned int masterActual = 0;//MASTER DEL CUAL SE ESTA LEYENDO LOS VALORES
bool masterButtonPress = false;//ESTADO DE BOTON DE COMANDO DE MASTER

unsigned int genScreen[60];//VARIABLES PARA LA PANTALLA DE GEN
unsigned int genActual = 1;//GEN DEL CUAL SE ESTA LEYENDO LOS VALORES
bool genButtonPress = false;//ESTADO DE BOTON DE COMANDO DE GEN
unsigned int priorityChange[4];//PRIORIDAD DE LOS GEN

unsigned int busScreen[50];//VARIABLES PARA LA PANTALLA DE BUS

//////////////////////////////////////////////////////
//MODOS DE LECTURA PARA HACER QUE SOLO LEA MASTERS O GENERADORES
int modoLectura = READ_MASTER_AND_GEN;

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
IPAddress servers[7]={
  IPAddress(192, 168, 137,  126),//MASTER1
  IPAddress(192, 168, 137,  128),//GEN1
  IPAddress(192, 168, 137,  126),//MASTER2
  IPAddress(192, 168, 137,  126),//MASTER3
  IPAddress(192, 168, 137,  126),//MASTER4
  IPAddress(192, 168, 137,  128),//GEN2
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



////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////SETUP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //////////////////////////////////////////////////////
  //UTILIDADES
  frameEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  frameEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  //////////////////////////////////////////////////////
  //UTILIDADES
  dataWriteSD.reserve(30);//RESERVANDO ESPACIO PARA EL STRING
  dataWriteSD = F("123456789012345678901234567890");//INICIANDO EL STRING CON VALORES ALEATORIOS

  asignAlarm();//ASINGANDO EL TEXTO DE LAS ALARMAS AL ARRAY DSEAlarmsString. FUNCION Y ARRAY DE DSEAlarms.h

  pinMode(LED_BUILTIN,OUTPUT);//LED FRONTAL
  pinMode(SWITCH_BUILTIN,INPUT);//SWITCH FRONTAL
  Serial.begin(115200);//COMUNICACION SERIAL
  delay(2000);//RETARDO PARA EL INICIO DEL PROGRAMA

  // if(debug){
    Serial.println(F("\n**********INIT**********"));//MARCANDO EL INICIO DEL PROGRAMA
  // }
  SD_Begin();//INICIANDO MICRO SD
  Serial.println(F("> Iniciando RTC"));
  rtc.begin();///INICIANDO RTC

  //////////////////////////////////////////////////////
  //COMFIGURANDO TIMER DE ACTUALIZACION DE FECHAS
  updateDateEvent.repeat();//EL TIMER SE REINICIA AUTOMATICAMENTE
  updateDateEvent.start();//EL TIMER INICIA DESDE QUE INICIA EL PROGRAMA

  readDseTimer.repeat();
  readDseTimer.start();
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
  if(!modbusTCPServer.begin()){//SI NO SE INICIA EL SERVIDOR MODBUS SE BLOQUEA EL PLC
    Serial.println(F("> Fallo al iniciar servidor Modbus"));
    //ACTIVAR LED DE ERROR
    while(1);
  }else{//SI SE INICIA EL SERVIDOR MODBUS
    // if(debug){
      Serial.print(F("> Servidor Modbus iniciado, IP: "));
      Serial.println(ip);
    // }
  }
  //////////////////////////////////////////////////////
  //CONFIGURANDO REGISTROS MODBUS
  modbusTCPServer.configureDiscreteInputs(0X00,200);
  modbusTCPServer.configureCoils(0x00,200);
  modbusTCPServer.configureInputRegisters(0x00,10);
  modbusTCPServer.configureHoldingRegisters(0x00,1600);
  Serial.println(F("> Registros Modbus configurados"));

  initializeArrays();//inicializando los arrays

  connectModules();
  readModuleDate();

  SD.remove(F("ACTIVE.csv"));//BORRANDO LAS ALARMAS ACTIVAS
  dataloggerInit();//INICIANDO EL DATALOGGER
  dataWriteSD = F("PLC REINICIADO");
  datalogger();
  dataloggerRead(rtc.getMonth(),rtc.getYear());//LEYENDO EL DATALOGGER DEL MES Y AÑO ACTUAL
  modbusTCPServer.holdingRegisterWrite(1240,rtc.getMonth());
  modbusTCPServer.holdingRegisterWrite(1241,rtc.getYear());
  Serial.println(F("> Setup finalizado"));

}//FIN SETUP

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////LOOP////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //LOGICA PARA BUS LIVE Y LA ALARMA COMUN GENERAL
  //para los master se toma dseInputs[d][1] y para los gen dseInputs[d][2]
  busLive = dseInputs[0][1] || dseInputs[1][2] || dseInputs[2][1] || dseInputs[3][1] || dseInputs[4][1] || dseInputs[5][2] || dseInputs[6][2] || dseInputs[7][2];
  generalCommonAlarm = gen1CommonAlarm || gen2CommonAlarm || gen3CommonAlarm || gen4CommonAlarm || master1CommonAlarm || master2CommonAlarm || master3CommonAlarm || master4CommonAlarm;


  if(readDseTimer.run()){//TIMER DE LECTURA DE LOS REGISTROS DE LOS DSE
    readDse();//LEYENDO LAS LOS REGISTROS DE ALARMAS DE LOS DSE
    computeDseAlarms();//SEPARANDO LAS ALARMAS QUE VIENEN EN EL MISMO REGISTRO
    activateAlarms();//DETERMINANDO SE SE HA ACTIVADO UNA ALARMA
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));//PAPADEO DEL LED FRONTAL
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
    dataloggerRead(monthTable,yearTable);//MOSTRANDO EL EVENTLOG
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
    alarmsLoggerRead();//MOSTRANDO LAS ALARMAS ACTIVAS
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
  kontrol.addListener("connectmodules",connectModules);
  kontrol.addListener("printmemory",printMemory);
  kontrol.addListener("readmode",modoLecturaCallback);
  kontrol.addListener("test",testCallback);

  //////////////////////////////////////////////////////
  //TIMER DE RECONEXION
  if(dseReconnect.run()){
    for(int i=0; i<NUMBER_OF_DSE; i++){
      dseErrorComm[i]=false;//SE DESACTIVA EL ERROR DE CONEXION Y SE INTENTA LA CONEXION
    }
    connectModules();
  }

  //////////////////////////////////////////////////////
  //ACTUALIZANDO LA FECHA DE LOS MODULOS Y DEL PLC
  if(updateDateEvent.run()){//TIMER DE ACTUALIZACION DE FECHA DE LOS MODULOS
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
    updateDseDates();
    Serial.println(F("> Fecha de modulos actualizadas"));
  }

  //////////////////////////////////////////////////////
  if(rtcUpdate.run()){//TIMER DE ACTUALIZACION DE RTC
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
