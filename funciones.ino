void initializeArrays(){//FUNCION QUE INICIALIZA LOS ARRAYS
  Serial.println(F("> Arrays inicializados"));
  eraseErrorComm();
  for(int i=0;i<NUMBER_OF_DSE;i++){
    oldDseErrorComm[i]=false;
    for(int j=0;j<150;j++){
      oldDseAlarms[i][j] =false;
    }
  }
}

void eraseErrorComm(){
  for(int i=0; i<NUMBER_OF_DSE; i++){
    dseErrorComm[i]=false;//SE DESACTIVA EL ERROR DE CONEXION Y SE INTENTA LA CONEXION
  }
}

void applyReadMode(){
  for(int d=0;d<NUMBER_OF_DSE;d++){
    switch (modoLectura){//SE CONECTAN LOS MODULOS DE ACUERDO AL MODO DE LECTURA
      case READ_MASTER_AND_GEN:
      dseBase = DSE_BASE_8660;
      break;

      case READ_ONLY_GEN:
      dseBase = DSE_BASE_8610;
      if(modulos[d].model == DSE_8660MKII){//
        dseErrorComm[d]=true;
        continue;
      }
      break;

      case READ_ONLY_MASTER:
      dseBase = DSE_BASE_8660;
      if(modulos[d].model == DSE_8610MKII){
        dseErrorComm[d]=true;
        continue;
      }
      break;
    }
  }
}

void handleModbusClients(){//funcion que maneja la conexion de los clientes
  EthernetClient newClient = server.accept(); //listen for incoming clients
  if (newClient) { //process new connection if possible
  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) { //Eight connections possible, find first available.
    if (!clients[i]) {
      clients[i] = newClient;
      client_cnt++;
      Serial.print(F("> Cliente Modbus conectado: ")); //a new client connected
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
      readModbusServerCoils();
      readModbusServerHoldingRegisters();
    }
  }

  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) { // Stop any clients which are disconnected
    if (clients[i] && !clients[i].connected()) {
      Serial.print(F("> Cliente Modbus desconectado: "));
      Serial.print(clients[i].remoteIP());
      Serial.print(F(", Total: "));
      clients[i].stop();
      client_cnt--;
      Serial.println(client_cnt);
    }
  }
}

void writeStringToRegisters(char _string[], unsigned int _add, unsigned int _size){
  for(int i=0;i<_size;i++){
    modbusTCPServer.holdingRegisterWrite(_add+i,_string[i*2]<<8 | _string[(i*2)+1]);
  }
}

void readDse(){//esta funcion lee los registros de los DSE
  for(int i=0;i<NUMBER_OF_DSE;i++){
    if(dseErrorComm[i]){//SI HAY UN ERROR DE CONEXION CON EL DSE ACTUAL SE SIGUE CON EL SIGUIENTE
      continue;
    }
    if(modulos[i].connect()){
      modulos[i].update();

      if(masterButtonPress && i == masterActual){//SI SE PRESIONA UN BOTON DE COMANDO ESTE SE ENVIA AL MODULO DSE
        if(modulos[masterActual].beginTransmission(4104,2)){
          modulos[masterActual].modbusWrite(masterScreen[59]);
          modulos[masterActual].modbusWrite(masterScreen[58]);
          modulos[masterActual].endTransmission();
          Serial.print(F("> "));
          Serial.print(modulos[masterActual].getName());
          Serial.println(F(" system key pressed"));
        } else{
          dseErrorComm[masterActual]=true;
        }
        masterButtonPress = false;
        modbusTCPServer.coilWrite(1,masterButtonPress);
      }

      if(genButtonPress && i== genActual){//SI SE PRESIONA UN BOTON DE COMANDO ESTE SE ENVIA AL MODULO DSE
        if(modulos[genActual].beginTransmission(4104,2)){
          modulos[genActual].modbusWrite(genScreen[55]);
          modulos[genActual].modbusWrite(genScreen[54]);
          modulos[genActual].endTransmission();
          Serial.print(F("> "));
          Serial.print(modulos[genActual].getName());
          Serial.println(F(" system key pressed"));
        } else{
          dseErrorComm[genActual]=true;
        }
        genButtonPress = false;
        modbusTCPServer.coilWrite(2,masterButtonPress);
      }

      //CAMBIO DE PRIORIDAD
      if(modoLectura == READ_MASTER_AND_GEN || modoLectura == READ_ONLY_GEN){
        if(modulos[i].model == DSE_8610MKII){//cambiando prioridad gen
          unsigned int add=0;
          switch(i){
            case 1:
            add=733;
            break;

            case 5:
            add=734;
            break;

            case 6:
            add=735;
            break;
          }
          unsigned int newPriority = constrain(modbusTCPServer.holdingRegisterRead(add),1,32);
          if(modulos[i].priority != newPriority){//SI LA PRIORIDAD ES DIFERENTE ENTONES SE CAMBIA
            if(modulos[i].beginTransmission(35104,1)){
              modulos[i].modbusWrite(newPriority);
              modulos[i].endTransmission();
              modulos[i].priority = newPriority;
            }
          }
          modbusTCPServer.holdingRegisterWrite(add,modulos[i].priority);
        }
      }

      modulos[i].stop();
      writeStringToRegisters(modulos[i].getName(),109+(i*20),6);
    }
    else{
      Serial.print(F("> Error trying to connect to: "));
      Serial.println(modulos[i].getName());
      dseErrorComm[i]= true;
      writeStringToRegisters("No Conected",109+(i*20),6);
    }
  }
}

void readModuleDate(){//FUNCION QUE LEE LA FECHA DEL MODULO ELEGIDO COMO BASE
  applyReadMode();
  //LEYENDO LA FECHA DEL MODULO BASE Y ACTUALIZANDO EL RTC
  if(!dseErrorComm[dseBase]){
    unsigned long time = modulos[dseBase].time;
    rtc.setEpoch(time);
    rtcUpdate.setFrecuency(RTC_UPDATE_TIME);
    Serial.print(F("> Actualizando RTC PLC\n\t"));
    Serial.println(getTime());
    Serial.print("\t");
    Serial.println(getDate());
    modbusTCPServer.holdingRegisterWrite(1240,rtc.getMonth());
    modbusTCPServer.holdingRegisterWrite(1241,rtc.getYear());
  } else {
    Serial.println(F("> Error al actualizar RTC. Intentando nuevamente en 10 segundos"));
  }
}

String twoDigits(long d){//funcion que muestra los numeros siempre con dos digitos
  if(d<10){
    return "0" + String(d);
  }
  return String(d);
}

String getTime(){//funcion que imprime la hora por serial
  return "Time: " + twoDigits(rtc.getHours()) + ":" + twoDigits(rtc.getMinutes()) + ":" + twoDigits(rtc.getSeconds());
}

String getDate(){//funcion que imprime la fecha por serial
  return "Date: " + twoDigits(rtc.getDay()) + "/" + twoDigits(rtc.getMonth()) + "/" + twoDigits(rtc.getYear());
}


void updateDseDates(){//FUNCION QUE ACTUALIZA LA FECHA DE LOS DSE TOMANDO UN MODULO BASE

  for(int i=0;i<NUMBER_OF_DSE;i++){

    switch (modoLectura) {
      case READ_MASTER_AND_GEN:
      if(i == dseBase){//saltando el dse base
        continue;
      }
      break;

      case READ_ONLY_GEN:
      if(i == dseBase || modulos[i].model == DSE_8660MKII){//saltando el dse base y los master
        continue;
      }
      break;

      case READ_ONLY_MASTER:
      if(i == dseBase || modulos[i].model == DSE_8610MKII){//saltando el dse base y los gen
        continue;
      }
      break;
    }

    if(!dseErrorComm[i] && !dseErrorComm[dseBase]){//REVISANDO SI EL MODULO BASE Y EL ACTUAL ESTAN CONECTADOS
      if(modulos[i].connect()){
        unsigned long time = modulos[dseBase].time;
        if(modulos[i].beginTransmission(1792,2)){//ESCRIBIENDO REGISTROS DE FECHA
          modulos[i].modbusWrite(time >> 16);
          modulos[i].modbusWrite(time);
          modulos[i].endTransmission();
          Serial.print(F("> RTC de "));
          Serial.print(modulos[i].getName());
          Serial.println(F(" actualizado"));
        }
        modulos[i].stop();
      }else {
        Serial.println(F("> Error al actualizar RTC de modulo"));
      }
    } else {
      Serial.println(F("> Error al actualizar RTC de modulo"));
    }
  }
}

void writeAlarmsLineModbus(unsigned int _reg,unsigned int _add){//FUNCION PARA ESCRIBIR UNA LINEA DE TEXTO EN LOS REGISTROS MODBUS
  int primeraComa = 0;//GUARDA EL INDICE DE LA PRIMERA COMA
  int segundaComa = 0;//GUARDA EL INDICE DE LA SEGUNDA COMA
  int final =0;//GUARDA EL INDICE DEL FINAL DE LA LINEA
  for(int i=0;i<50;i++){//BUSCANDO LA PRIMERA COMA
    if(alarmLine[i] == ','){
      primeraComa = i;
      break;
    }
  }
  for(int i=primeraComa+1;i<50;i++){//BUSCANDO LA SEGUNDA COMA
    if(alarmLine[i] == ','){
      segundaComa = i;
      break;
    }
  }
  for(int i=segundaComa+1;i<50;i++){//BUSCANDO EL FINAL DE LA LINEA
    if(alarmLine[i] == '\n'){
      final = i;
      break;
    }else{
      final =49;
    }
  }

  char buff[30];
  for (int i=0;i<30;i++){//LIMPIANDO BUFFER
    buff[i]=0;
  }
  for(int i=0;i<primeraComa;i++){//EXTRAYENDO EL TEXTO ANTERIOR A LA PRIMERA COMA
    buff[i] = alarmLine[i];
  }
  for(int j=0;j<5;j++){//ENVIANDO EL TEXTO POR MODBUS
    modbusTCPServer.holdingRegisterWrite(_add+j+(_reg*5),buff[j*2] <<8 | buff[j*2+1]);
  }

  for (int i=0;i<30;i++){//LIMPIANDO BUFFER
    buff[i]=0;
  }
  for(int i=primeraComa+1;i<segundaComa;i++){//EXTRAYENDO EL TEXTO ENTRE LAS COMAS
    buff[i-(primeraComa+1)] = alarmLine[i];
  }
  for(int j=0;j<5;j++){//ENVIANDO EL TEXTO POR MODBUS
    modbusTCPServer.holdingRegisterWrite(_add+50+j+(_reg*5),buff[j*2] <<8 | buff[j*2+1]);
  }

  for (int i=0;i<30;i++){//LIMPIANDO BUFFER
    buff[i]=0;
  }
  for(int i=segundaComa+1;i<final;i++){//EXTRAYENDO EL TEXTO DESPUES DE LA SEGUNDA COMA
    buff[i-(segundaComa+1)] = alarmLine[i];
  }
  for(int j=0;j<15;j++){//ENVIANDO EL TEXTO POR MODBUS
    modbusTCPServer.holdingRegisterWrite(_add+100+j+(_reg*15),buff[j*2] <<8 | buff[j*2+1]);
  }
}

void activateAlarms(){//FUNCION QUE DETERMINA SI ESTA ACTIVA UNA ALARMA Y LA GUARDA

  static bool alarmaDesactivada = false;

  if(alarmaDesactivada){
    //SI SE DESACTIVA UNA ALARMA SE BORRA EL ARCHIVO DE ALARMAS ACTIVAS
    //LUEGO SE VUELVE A BUSCAR LAS ALARMAS ACTIVAS Y SE GUARDAN
    SD.remove(F("ACTIVE.csv"));
    for(int i=0;i<NUMBER_OF_DSE;i++){//BUSCANDO ALARMAS DE ERROR DE CONEXION
      switch (modoLectura) {
        case READ_ONLY_GEN:
        if(modulos[i].model == DSE_8660MKII){//saltando los master
          continue;
        }
        break;

        case READ_ONLY_MASTER:
        if(modulos[i].model == DSE_8610MKII){//saltando los gen
          continue;
        }
        break;

        case READ_MASTER_AND_GEN:
        break;
      }
      if(oldDseErrorComm[i]){//si hubo un error de conexion
        dataWriteSD = modulos[i].getName();
        dataWriteSD += F(" COMM ERROR");
        alarmsLogger();
      }
      for (int j=0;j<150;j++){
        if(oldDseAlarms[i][j]){
          dataWriteSD = modulos[i].getName();
          dataWriteSD += " ";
          dataWriteSD += DSEAlarmsString[j-1];
          alarmsLogger();
        }
      }
    }
    alarmaDesactivada=false;
  }

  for(int i=0;i<NUMBER_OF_DSE;i++){//REVISANDO SI SE DESACTIVA UNA ALARMA DE ERROR DE CONEXION O DE DSE
    switch (modoLectura) {
      case READ_ONLY_GEN:
      if(modulos[i].model == DSE_8660MKII){//saltando los master
        continue;
      }
      break;

      case READ_ONLY_MASTER:
      if(modulos[i].model == DSE_8610MKII){//saltando los gen
        continue;
      }
      break;

      case READ_MASTER_AND_GEN:
      break;
    }
    if(oldDseErrorComm[i] && !dseErrorComm[i]){
      alarmaDesactivada =true;
      break;
    }
    for (int j=0;j<150;j++){
      if(oldDseAlarms[i][j] && !modulos[i].alarms[j]){
        alarmaDesactivada =true;
        break;
      }
      if(alarmaDesactivada){
        break;
      }
    }
  }

  for(int i=0;i<NUMBER_OF_DSE;i++){//REVISANDO SI SE ACTIVA UNA ALARMA DE ERROR DE CONEXION
    switch (modoLectura) {
      case READ_ONLY_GEN:
      if(modulos[i].model == DSE_8660MKII){//saltando los master
        continue;
      }
      break;

      case READ_ONLY_MASTER:
      if(modulos[i].model == DSE_8610MKII){//saltando los gen
        continue;
      }
      break;

      case READ_MASTER_AND_GEN:
      break;
    }
    if(!oldDseErrorComm[i] && dseErrorComm[i]){
      dataWriteSD = modulos[i].getName();
      dataWriteSD += F(" COMM ERROR");
      datalogger();//AGREGANDO AL EVENTO LOG
      alarmsLogger();//AGREGANDO A LAS ALARMAS ACTIVAS
    }
    oldDseErrorComm[i] = dseErrorComm[i];

    for (int j=0;j<150;j++){
      if(!oldDseAlarms[i][j] && modulos[i].alarms[j]){
        dataWriteSD = modulos[i].getName();
        dataWriteSD += " ";
        dataWriteSD += DSEAlarmsString[j-1];//SE TOMA EL TEXTO DE LA ALARMA DE DSEAlarms.H
        datalogger();//AGREGANDO AL EVENT LOG
        alarmsLogger();//AGREGANDO A LAS ALARMAS ACTIVAS
      }
      oldDseAlarms[i][j] = modulos[i].alarms[j];
    }
  }
}

void computeSchRegisters(){//FUNCION QUE HACE LOS CALCULOS DEL SCHEDULE
  //los siguientes "if" hacen que si se disminuye la cantidad menos que el limite inferior esta pasa a su limite superior
  // si es menor a 0 pasa a 23 y si es mayor a 23 pasa a 0
  //la forma en que esta implementado es porque la variable es sin signo osea que ninca va a ser menor a 0
  //cuando se disminuye desde 0 entonces pasa a ser un numero muy grande
  //por lo que solo basta detectar que sea un numero muy grande
  if(schHolding[0]>1000){//restringiendo las horas
    schHour = 23;
  } else if (schHolding[0]>23){
    schHour = 0;
  } else{
    schHour = schHolding[0];
  }

  if(schHolding[1]>1000){//restringiendo los minutos
    schMinute = 59;
  } else if (schHolding[1]>59){
    schMinute = 0;
  }else{
    schMinute = schHolding[1];
  }

  if(schHolding[3]<1){//restringiendo los meses
    schMonth = 12;
  } else if (schHolding[3]>12){
    schMonth = 1;
  }else{
    schMonth = schHolding[3];
  }

  if(schHolding[4]>50000){//duracion entre 0 y 30 dias (minutos)
    schDuration = 43200;
  } else if (schHolding[4]>43200){
    schDuration = 0;
  }else{
    schDuration = schHolding[4];
  }

  switch (schMonth){//se restringen los dias dependiendo el mes
    case 1://enero
    case 3://marzo
    case 5://mayo
    case 7://julio
    case 8://agosto
    case 10://octubre
    case 12://diciembre
    //se restringen los dias a 31 cuando son los meses correspondientes
    if(schHolding[2]<1){
      schDay = 31;
    } else if (schHolding[2]>31){
      schDay = 1;
    }else{
      schDay = schHolding[2];
    }
    break;

    case 2://febrero
    //se restringe a 28 dias si es febrero
    if(schHolding[2]<1){
      schDay = 28;
    } else if (schHolding[2]>28){
      schDay = 1;
    }else{
      schDay = schHolding[2];
    }
    break;

    case 4://abril
    case 6://junio
    case 9://septiembre
    case 11://noviembre
    //se restringe a 30 dias en los meses correspondientes
    if(schHolding[2]<1){
      schDay = 30;
    } else if (schHolding[2]>30){
      schDay = 1;
    }else{
      schDay = schHolding[2];
    }
    break;

    default:
    //se restringe a 30 dias por defecto
    if(schHolding[2]<1){
      schDay = 30;
    } else if (schHolding[2]>30){
      schDay = 1;
    }else{
      schDay = schHolding[2];
    }
    break;

  }
  //GUARDANDO RESULTADO EN EL ARRAY
  schHolding[0] = schHour;
  schHolding[1] = schMinute;
  schHolding[2] = schDay;
  schHolding[3] = schMonth;
  schHolding[4] = schDuration;

  //////////////////////////////////////////////////////
  //coils

  //enable
  schEnable = schCoils[7];

  //test off/on load
  if(schCoils[13] && schTestLoad == SCH_TEST_ON_LOAD){//cambiando a test off load
    schTestLoad = SCH_TEST_OFF_LOAD;
    schCoils[14]=false;
  }
  if(schCoils[14] && schTestLoad == SCH_TEST_OFF_LOAD){//cambiando a test on load
    schTestLoad = SCH_TEST_ON_LOAD;
    schCoils[13]=false;
  }

  //transition open/closed
  if(schCoils[15] && schTransition == SCH_TRANSITION_CLOSED){//cambiando a transition open
    schTransition = SCH_TRANSITION_OPEN;
    schCoils[16]=false;
  }
  if(schCoils[16] && schTransition == SCH_TRANSITION_OPEN){//cambiando a transition closed
    schTransition = SCH_TRANSITION_CLOSED;
    schCoils[15]=false;
  }
  //load demand Inhibit
  schLoadDemandInhibit = schCoils[17];

  //tipo repeticion
  if(schCoils[9] && schTipoRepeticion !=SCH_DAILY){//activar la repeticion diaria y desactivando las demas
    schTipoRepeticion = SCH_DAILY;
    schCoils[10] = false;
    schCoils[11] = false;
    schCoils[12] = false;
  }
  if(schCoils[10] && schTipoRepeticion !=SCH_WEEKLY){//activar la repeticion semanal y desactivando las demas
    schTipoRepeticion = SCH_WEEKLY;
    schCoils[9] = false;
    schCoils[11] = false;
    schCoils[12] = false;
  }
  if(schCoils[11] && schTipoRepeticion !=SCH_MONTHLY){//activar la repeticion mensual y desactivando las demas
    schTipoRepeticion = SCH_MONTHLY;
    schCoils[10] = false;
    schCoils[9] = false;
    schCoils[12] = false;
  }
  if(schCoils[12] && schTipoRepeticion !=SCH_DATE){//activar activacion en fecha y desactivando las demas
    schTipoRepeticion = SCH_DATE;
    schCoils[10] = false;
    schCoils[11] = false;
    schCoils[9] = false;
  }

  //computando si se activa el schedule
  if(schDuration == 0){
    schDurationTimer.setFrecuency(2000);//TIMER DE 2 SEGUNDOS PARA DESACTIVAR EL SCHEDULE
  }else{
    schDurationTimer.setFrecuency(schDuration*60*1000);
  }
  switch(schTipoRepeticion){

    case SCH_DAILY://en repeticion diaria solo se comprueba la hora y los minutos
    if(!schActive && schEnable && (rtc.getHours() == schHour) && (rtc.getMinutes() == schMinute) && (rtc.getSeconds() == 0)){
      schActive = true;
      schDurationTimer.start();
      Serial.print(F("> Schedule activado. Duracion: "));
      Serial.print(schDuration);
      Serial.println(F(" minutos"));
    }
    break;

    case SCH_WEEKLY:
    if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0){
      //0 jueves, 1 viernes, 2 sabado, 3 domingo, 4 lunes, 5 martes, 6 miercoles
      //rtc.getEpoch();
      unsigned long dayOfWeek= (rtc.getEpoch() / 86400) % 7;
      //calculando se el dia corresponde a alguno de los elegidos
      bool diaActivo = (schCoils[0] && dayOfWeek == 4) || (schCoils[1] && dayOfWeek == 5) || (schCoils[2] && dayOfWeek == 6) || (schCoils[3] && dayOfWeek == 0) || (schCoils[4] && dayOfWeek == 1) || (schCoils[5] && dayOfWeek == 2) || (schCoils[6] && dayOfWeek == 3);
      if(diaActivo){
        schActive = true;
        schDurationTimer.start();
        Serial.println(F("Schedule activado"));
      }
    }
    break;

    case SCH_MONTHLY:
    if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0 && rtc.getDay() == schDay){
      schActive = true;
      schDurationTimer.start();
      Serial.println(F("Schedule activado"));
    }
    break;

    case SCH_DATE:
    if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0 && rtc.getDay() == schDay && rtc.getMonth() == schMonth){
      schActive = true;
      schDurationTimer.start();
      Serial.println(F("Schedule activado"));
    }
    break;
  }

  schCoils[8] = schActive;

}


////MODBUS
void readModbusServerCoils(){//FUNCION QUE LEE LOS COILS
  updateModulesDates = modbusTCPServer.coilRead(0);
  masterButtonPress = modbusTCPServer.coilRead(1);
  genButtonPress = modbusTCPServer.coilRead(2);

  // gen1CommonAlarm = modbusTCPServer.coilRead(21);
  // gen2CommonAlarm = modbusTCPServer.coilRead(22);
  // gen3CommonAlarm = modbusTCPServer.coilRead(23);
  // gen4CommonAlarm = modbusTCPServer.coilRead(24);
  // master1CommonAlarm = modbusTCPServer.coilRead(25);
  // master2CommonAlarm = modbusTCPServer.coilRead(26);
  // master3CommonAlarm = modbusTCPServer.coilRead(27);
  // master4CommonAlarm = modbusTCPServer.coilRead(28);

  //leyendo los coils del schedule
  for(int i=0; i<18;i++){
    schCoils[i] = modbusTCPServer.coilRead(i+30);
  }
}

void writeModbusCoils(){//FUNCION QUE ESCRIBE LOS COILS
  modbusTCPServer.coilWrite(0,updateModulesDates);
  modbusTCPServer.coilWrite(1,masterButtonPress);
  modbusTCPServer.coilWrite(2,genButtonPress);
  modbusTCPServer.coilWrite(10,busLive);
  modbusTCPServer.coilWrite(20,generalCommonAlarm);
  //escribiendo registros del schedule
  for(int i=0; i<18;i++){
    modbusTCPServer.coilWrite(i+30,schCoils[i]);
  }
  //ESCRIBIENDO LOS ERRORES DE COMUNICACION
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.coilWrite(i+50,dseErrorComm[i]);
  }
}
//
void writeModbusDiscreteInputs() {//FUNCION QUE ESCRIBE LAS ENTRADAS DISCRETAS
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.discreteInputWrite((i*10),modulos[i].mainsAvailable);
    if(modulos[i].model == DSE_8660MKII){
      modbusTCPServer.discreteInputWrite(1+(i*10),modulos[i].busAvailable);
      modbusTCPServer.discreteInputWrite(2+(i*10),modulos[i].mainBrk);
    }
    if(modulos[i].model == DSE_8610MKII){
      modbusTCPServer.discreteInputWrite(1+(i*10),modulos[i].genAvailable);
      modbusTCPServer.discreteInputWrite(2+(i*10),modulos[i].genBrk);
    }
    modbusTCPServer.discreteInputWrite(3+(i*10),modulos[i].busBrk);
    modbusTCPServer.discreteInputWrite(4+(i*10),modulos[i].loadOn);
  }
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.discreteInputWrite(100+i,dseErrorComm[i]);
  }
}
//
void writeModbusInputRegisters() {//FUNCION QUE ESCRIBE LOS INPUTS

}

void readModbusServerHoldingRegisters(){//FUNCION QUE LEE LOS HOLDING

  for(int i=0;i<5;i++){//leyendo registros del schedule
    schHolding[i]= modbusTCPServer.holdingRegisterRead(i+10);
  }
  masterActual = modbusTCPServer.holdingRegisterRead(499);
  masterScreen[58]= modbusTCPServer.holdingRegisterRead(558);
  masterScreen[59]= modbusTCPServer.holdingRegisterRead(559);
  genActual = modbusTCPServer.holdingRegisterRead(599);
  genScreen[54]= modbusTCPServer.holdingRegisterRead(654);
  genScreen[55]= modbusTCPServer.holdingRegisterRead(655);

  if(masterActual != 0 && masterActual != 2 && masterActual != 3 && masterActual != 4){//ASEGURANDO QUE EL MASTER ACTUAL SEA UNO CORRECTO
    masterActual=0;
  }

  if(genActual != 1 && genActual != 5 && genActual !=6){//ASEGURANDO QUE EL GEN ACTUAL SEA UNO CORRECTO
    genActual=1;
  }
  modbusTCPServer.holdingRegisterWrite(733,constrain(modbusTCPServer.holdingRegisterRead(733),1,32));
  modbusTCPServer.holdingRegisterWrite(734,constrain(modbusTCPServer.holdingRegisterRead(734),1,32));
  modbusTCPServer.holdingRegisterWrite(735,constrain(modbusTCPServer.holdingRegisterRead(735),1,32));
}
//
void writeModbusHoldingRegisters(){//FUNCION QUE ESCRIBE LOS HOLDING
  //escribiendo registros del schedule
  for(int i=0;i<5;i++){
    modbusTCPServer.holdingRegisterWrite(i+10,schHolding[i]);
  }
  //ESCRIBIENDO REGISTROS DE LA PANTALLA PRINCIPAL
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.holdingRegisterWrite(100+(i*20),modulos[i].HZ);
    modbusTCPServer.holdingRegisterWrite(101+(i*20),modulos[i].V>>16);
    modbusTCPServer.holdingRegisterWrite(102+(i*20),modulos[i].V);
    modbusTCPServer.holdingRegisterWrite(103+(i*20),modulos[i].KW>>16);
    modbusTCPServer.holdingRegisterWrite(104+(i*20),modulos[i].KW);
    modbusTCPServer.holdingRegisterWrite(105+(i*20),modulos[i].P);
    modbusTCPServer.holdingRegisterWrite(106+(i*20),modulos[i].KVAR>>16);
    modbusTCPServer.holdingRegisterWrite(107+(i*20),modulos[i].KVAR);
    modbusTCPServer.holdingRegisterWrite(108+(i*20),modulos[i].mode);
  }

  //ESCRIBIENDO LOS REGISTROS DE LA PANTALLA DE MASTER
  if(dseErrorComm[masterActual]){
    for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de master
        modbusTCPServer.holdingRegisterWrite(500+i,0);
    }
    writeStringToRegisters("No Conected",552,6);
  } else {
    modbusTCPServer.holdingRegisterWrite(500,modulos[masterActual].mainsAvailable);
    modbusTCPServer.holdingRegisterWrite(501,modulos[masterActual].busAvailable);
    modbusTCPServer.holdingRegisterWrite(502,modulos[masterActual].mainBrk);
    modbusTCPServer.holdingRegisterWrite(503,modulos[masterActual].busBrk);
    modbusTCPServer.holdingRegisterWrite(504,modulos[masterActual].loadOn);
    modbusTCPServer.holdingRegisterWrite(505,modulos[masterActual].LLAVR>>16);
    modbusTCPServer.holdingRegisterWrite(506,modulos[masterActual].LLAVR);
    modbusTCPServer.holdingRegisterWrite(507,modulos[masterActual].HZ);
    modbusTCPServer.holdingRegisterWrite(508,modulos[masterActual].KW>>16);
    modbusTCPServer.holdingRegisterWrite(509,modulos[masterActual].KW);
    modbusTCPServer.holdingRegisterWrite(510,modulos[masterActual].P);
    modbusTCPServer.holdingRegisterWrite(511,modulos[masterActual].busLLAVR>>16);
    modbusTCPServer.holdingRegisterWrite(512,modulos[masterActual].busLLAVR);
    modbusTCPServer.holdingRegisterWrite(513,modulos[masterActual].busHZ);
    modbusTCPServer.holdingRegisterWrite(514,modulos[masterActual].busP);
    modbusTCPServer.holdingRegisterWrite(515,modulos[masterActual].L1N>>16);
    modbusTCPServer.holdingRegisterWrite(516,modulos[masterActual].L1N);
    modbusTCPServer.holdingRegisterWrite(517,modulos[masterActual].L2N>>16);
    modbusTCPServer.holdingRegisterWrite(518,modulos[masterActual].L2N);
    modbusTCPServer.holdingRegisterWrite(519,modulos[masterActual].L3N>>16);
    modbusTCPServer.holdingRegisterWrite(520,modulos[masterActual].L3N);
    modbusTCPServer.holdingRegisterWrite(521,modulos[masterActual].L1L2>>16);
    modbusTCPServer.holdingRegisterWrite(522,modulos[masterActual].L1L2);
    modbusTCPServer.holdingRegisterWrite(523,modulos[masterActual].L2L3>>16);
    modbusTCPServer.holdingRegisterWrite(524,modulos[masterActual].L2L3);
    modbusTCPServer.holdingRegisterWrite(525,modulos[masterActual].L3L1>>16);
    modbusTCPServer.holdingRegisterWrite(526,modulos[masterActual].L3L1);
    modbusTCPServer.holdingRegisterWrite(527,modulos[masterActual].IL1>>16);
    modbusTCPServer.holdingRegisterWrite(528,modulos[masterActual].IL1);
    modbusTCPServer.holdingRegisterWrite(529,modulos[masterActual].IL2>>16);
    modbusTCPServer.holdingRegisterWrite(530,modulos[masterActual].IL2);
    modbusTCPServer.holdingRegisterWrite(531,modulos[masterActual].IL3>>16);
    modbusTCPServer.holdingRegisterWrite(532,modulos[masterActual].IL3);
    modbusTCPServer.holdingRegisterWrite(533,modulos[masterActual].busL1N>>16);
    modbusTCPServer.holdingRegisterWrite(534,modulos[masterActual].busL1N);
    modbusTCPServer.holdingRegisterWrite(535,modulos[masterActual].busL2N>>16);
    modbusTCPServer.holdingRegisterWrite(536,modulos[masterActual].busL2N);
    modbusTCPServer.holdingRegisterWrite(537,modulos[masterActual].busL3N>>16);
    modbusTCPServer.holdingRegisterWrite(538,modulos[masterActual].busL3N);
    modbusTCPServer.holdingRegisterWrite(539,modulos[masterActual].busL1L2>>16);
    modbusTCPServer.holdingRegisterWrite(540,modulos[masterActual].busL1L2);
    modbusTCPServer.holdingRegisterWrite(541,modulos[masterActual].busL2L3>>16);
    modbusTCPServer.holdingRegisterWrite(542,modulos[masterActual].busL2L3);
    modbusTCPServer.holdingRegisterWrite(543,modulos[masterActual].busL3L1>>16);
    modbusTCPServer.holdingRegisterWrite(544,modulos[masterActual].busL3L1);
    modbusTCPServer.holdingRegisterWrite(545,modulos[masterActual].PF);
    modbusTCPServer.holdingRegisterWrite(546,modulos[masterActual].KVA>>16);
    modbusTCPServer.holdingRegisterWrite(547,modulos[masterActual].KVA);
    modbusTCPServer.holdingRegisterWrite(548,modulos[masterActual].KVAR>>16);
    modbusTCPServer.holdingRegisterWrite(549,modulos[masterActual].KVAR);
    modbusTCPServer.holdingRegisterWrite(550,modulos[masterActual].PhRot);
    modbusTCPServer.holdingRegisterWrite(551,modulos[masterActual].mode);
    writeStringToRegisters(modulos[masterActual].getName(),552,6);
  }

  //ESCRIBIENDO LOS REGISTROS DE LA PANTALLA DE GEN
  if(dseErrorComm[genActual]){
    for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de master
        modbusTCPServer.holdingRegisterWrite(600+i,0);
    }
    writeStringToRegisters("No Conected",648,6);
  } else {
    modbusTCPServer.holdingRegisterWrite(600,modulos[genActual].genAvailable);
    modbusTCPServer.holdingRegisterWrite(601,modulos[genActual].genBrk);
    modbusTCPServer.holdingRegisterWrite(602,modulos[genActual].LLAVR>>16);
    modbusTCPServer.holdingRegisterWrite(603,modulos[genActual].LLAVR);
    modbusTCPServer.holdingRegisterWrite(604,modulos[genActual].HZ);
    modbusTCPServer.holdingRegisterWrite(605,modulos[genActual].KW>>16);
    modbusTCPServer.holdingRegisterWrite(606,modulos[genActual].KW);
    modbusTCPServer.holdingRegisterWrite(607,modulos[genActual].P);
    modbusTCPServer.holdingRegisterWrite(608,modulos[genActual].busLLAVR>>16);
    modbusTCPServer.holdingRegisterWrite(609,modulos[genActual].busLLAVR);
    modbusTCPServer.holdingRegisterWrite(610,modulos[genActual].busHZ);
    modbusTCPServer.holdingRegisterWrite(611,modulos[genActual].busP);
    modbusTCPServer.holdingRegisterWrite(612,modulos[genActual].L1N>>16);
    modbusTCPServer.holdingRegisterWrite(613,modulos[genActual].L1N);
    modbusTCPServer.holdingRegisterWrite(614,modulos[genActual].L2N>>16);
    modbusTCPServer.holdingRegisterWrite(615,modulos[genActual].L2N);
    modbusTCPServer.holdingRegisterWrite(616,modulos[genActual].L3N>>16);
    modbusTCPServer.holdingRegisterWrite(617,modulos[genActual].L3N);
    modbusTCPServer.holdingRegisterWrite(618,modulos[genActual].L1L2>>16);
    modbusTCPServer.holdingRegisterWrite(619,modulos[genActual].L1L2);
    modbusTCPServer.holdingRegisterWrite(620,modulos[genActual].L2L3>>16);
    modbusTCPServer.holdingRegisterWrite(621,modulos[genActual].L2L3);
    modbusTCPServer.holdingRegisterWrite(622,modulos[genActual].L3L1>>16);
    modbusTCPServer.holdingRegisterWrite(623,modulos[genActual].L3L1);
    modbusTCPServer.holdingRegisterWrite(624,modulos[genActual].IL1>>16);
    modbusTCPServer.holdingRegisterWrite(625,modulos[genActual].IL1);
    modbusTCPServer.holdingRegisterWrite(626,modulos[genActual].IL2>>16);
    modbusTCPServer.holdingRegisterWrite(627,modulos[genActual].IL2);
    modbusTCPServer.holdingRegisterWrite(628,modulos[genActual].IL3>>16);
    modbusTCPServer.holdingRegisterWrite(629,modulos[genActual].IL3);
    modbusTCPServer.holdingRegisterWrite(630,modulos[genActual].PF);
    modbusTCPServer.holdingRegisterWrite(631,modulos[genActual].KVA>>16);
    modbusTCPServer.holdingRegisterWrite(632,modulos[genActual].KVA);
    modbusTCPServer.holdingRegisterWrite(633,modulos[genActual].KVAR>>16);
    modbusTCPServer.holdingRegisterWrite(634,modulos[genActual].KVAR);
    modbusTCPServer.holdingRegisterWrite(635,modulos[genActual].PhRot);
    modbusTCPServer.holdingRegisterWrite(636,modulos[genActual].oilPressure);
    modbusTCPServer.holdingRegisterWrite(637,modulos[genActual].battery);
    modbusTCPServer.holdingRegisterWrite(638,modulos[genActual].engineSpeed);
    modbusTCPServer.holdingRegisterWrite(639,modulos[genActual].coolantTemperature);
    modbusTCPServer.holdingRegisterWrite(640,modulos[genActual].fuelLevel);
    modbusTCPServer.holdingRegisterWrite(641,modulos[genActual].engineRuntime>>16);
    modbusTCPServer.holdingRegisterWrite(642,modulos[genActual].engineRuntime);
    modbusTCPServer.holdingRegisterWrite(643,modulos[genActual].numberOfStarts>>16);
    modbusTCPServer.holdingRegisterWrite(644,modulos[genActual].numberOfStarts);
    modbusTCPServer.holdingRegisterWrite(645,modulos[genActual].KWH>>16);
    modbusTCPServer.holdingRegisterWrite(646,modulos[genActual].KWH);
    modbusTCPServer.holdingRegisterWrite(647,modulos[genActual].mode);
    writeStringToRegisters(modulos[genActual].getName(),648,6);
  }
  //ESCRIBIENDO REGISTROS DE LA PANTALLA DE BUS
  //PANTALLA DE BUS. SOLO SALE DE LOS GEN
  if(modoLectura == READ_ONLY_GEN|| modoLectura == READ_MASTER_AND_GEN){
    for(int i=0;i<31;i++){
        modbusTCPServer.holdingRegisterWrite(700+i,0);
    }

    unsigned long currentL1=0;
    unsigned long currentL2=0;
    unsigned long currentL3=0;
    unsigned long totalWattsL1=0;
    unsigned long totalWattsL2=0;
    unsigned long totalWattsL3=0;

    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(modulos[i].model == DSE_8660MKII){
        continue;
      }
      if(!dseErrorComm[i]){
        currentL1 += modulos[i].IL1;
        currentL2 += modulos[i].IL2;
        currentL3 += modulos[i].IL3;

        // totalWattsL1 += modulos[i].KW;//bus total watts l1
        // totalWattsL2 += modulos[i].KW;//bus total watts l2
        // totalWattsL3 += modulos[i].KW;//
      }
    }
    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(modulos[i].model == DSE_8660MKII){
        continue;
      }
      if(!dseErrorComm[i]){
        modbusTCPServer.holdingRegisterWrite(700,modulos[i].busKW>>16);
        modbusTCPServer.holdingRegisterWrite(701,modulos[i].busKW);
        modbusTCPServer.holdingRegisterWrite(702,modulos[i].busLLAVR>>16);
        modbusTCPServer.holdingRegisterWrite(703,modulos[i].busLLAVR);
        modbusTCPServer.holdingRegisterWrite(704,modulos[i].busLNAVR>>16);
        modbusTCPServer.holdingRegisterWrite(705,modulos[i].busLNAVR);
        modbusTCPServer.holdingRegisterWrite(706,modulos[i].busP);
        modbusTCPServer.holdingRegisterWrite(707,modulos[i].busL1N>>16);
        modbusTCPServer.holdingRegisterWrite(708,modulos[i].busL1N);
        modbusTCPServer.holdingRegisterWrite(709,modulos[i].busL2N>>16);
        modbusTCPServer.holdingRegisterWrite(710,modulos[i].busL2N);
        modbusTCPServer.holdingRegisterWrite(711,modulos[i].busL3N>>16);
        modbusTCPServer.holdingRegisterWrite(712,modulos[i].busL3N);
        modbusTCPServer.holdingRegisterWrite(713,modulos[i].busL1L2>>16);
        modbusTCPServer.holdingRegisterWrite(714,modulos[i].busL1L2);
        modbusTCPServer.holdingRegisterWrite(715,modulos[i].busL2L3>>16);
        modbusTCPServer.holdingRegisterWrite(716,modulos[i].busL2L3);
        modbusTCPServer.holdingRegisterWrite(717,modulos[i].busL3L1>>16);
        modbusTCPServer.holdingRegisterWrite(718,modulos[i].busL3L1);
        modbusTCPServer.holdingRegisterWrite(719,currentL1>>16);
        modbusTCPServer.holdingRegisterWrite(720,currentL1);
        modbusTCPServer.holdingRegisterWrite(721,currentL2>>16);
        modbusTCPServer.holdingRegisterWrite(722,currentL2);
        modbusTCPServer.holdingRegisterWrite(723,currentL3>>16);
        modbusTCPServer.holdingRegisterWrite(724,currentL3);
        modbusTCPServer.holdingRegisterWrite(725,totalWattsL1>>16);
        modbusTCPServer.holdingRegisterWrite(726,totalWattsL1);
        modbusTCPServer.holdingRegisterWrite(727,totalWattsL2>>16);
        modbusTCPServer.holdingRegisterWrite(728,totalWattsL2);
        modbusTCPServer.holdingRegisterWrite(729,totalWattsL3>>16);
        modbusTCPServer.holdingRegisterWrite(730,totalWattsL3);
        modbusTCPServer.holdingRegisterWrite(731,modulos[i].busHZ);
        modbusTCPServer.holdingRegisterWrite(732,modulos[i].busPhRot);

        modbusTCPServer.holdingRegisterWrite(741,modulos[i].mastersOnline);
        modbusTCPServer.holdingRegisterWrite(742,modulos[i].gensOnline);
        break;
      }
    }
    // PRIORIDAD Y QUALITY
    if(!dseErrorComm[1]){
      // modbusTCPServer.holdingRegisterWrite(733,modulos[1].priority);
      modbusTCPServer.holdingRegisterWrite(737,modulos[1].qualityMSC);
    }
    if(!dseErrorComm[5]){
      // modbusTCPServer.holdingRegisterWrite(734,modulos[5].priority);
      modbusTCPServer.holdingRegisterWrite(738,modulos[5].qualityMSC);
    }
    if(!dseErrorComm[6]){
      // modbusTCPServer.holdingRegisterWrite(735,modulos[6].priority);
      modbusTCPServer.holdingRegisterWrite(739,modulos[6].qualityMSC);
    }
  }
  //RESTRINGIENDO EL VALOR DEL MES A MOSTRAR EN EL EVENT LOG
  unsigned int monthServer = modbusTCPServer.holdingRegisterRead(1240);
  unsigned int yearServer = modbusTCPServer.holdingRegisterRead(1241);
  if(monthServer>12){//SI SE AUMENTA EL MES 12 ENTONCES SE SUMA UN AÑO
    monthServer =1;
    yearServer++;
  }
  if(monthServer<1){//SI SE DISMINUYE EL MES 1 ENTONCES SE RESTA UN AÑO
    monthServer =12;
    yearServer--;
  }

  modbusTCPServer.holdingRegisterWrite(1240,monthServer);
  modbusTCPServer.holdingRegisterWrite(1241,yearServer);

}
//
//
void utilidades(){
  //CALCULANDO EL TIEMPO DE EJECUCION DE UN CICLO DE PROGRAMA (FRAME)
  static unsigned int frameNumbers =0;
  frame += micros() - beforeFrame;
  beforeFrame = micros();
  frameNumbers ++;

  if(frameEvent.run() && debugUtilidades){

    modoLecturaCallback();//IMPRIMIENDO EL MODO DE LECTURA

    //IMPRIMIENDO EL TIEMPO DE FRAME
    Serial.print(F("> Frame time(us): "));
    Serial.println(frame/frameNumbers);

    //IMPRIMIENDO LA MEMORIA DISPONIBLE
    printMemory();

    Serial.println();
    frame = frameNumbers = 0;
  }
}

void printMemory(){//FUNCION QUE IMPRIME POR SERIAL LA MEMORIA DISPONIBLE
  Serial.print(F("> MEM FREE: "));
  Serial.print(freeMemory(), DEC);
  Serial.print(", ");
  float percent = freeMemory();
  percent = percent*100/32000;
  Serial.print(percent,2);
  Serial.println("%");
}

void test(){//FUNCION DE PRUEBA

}
