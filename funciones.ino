void initializeArrays(){
  if(debug){Serial.println(F("> Arrays inicializados"));}
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

void connectModules(){
  for(int d=0;d<NUMBER_OF_DSE;d++){
    if (!modbusTCPClient[d].connected()) {// client not connected, start the Modbus TCP client
      if(debug){
        Serial.print(F("> Intentando conectar al servidor Modbus con IP:"));
        Serial.println(servers[d]);
      }

      if (!modbusTCPClient[d].begin(servers[d])) {
        if(debug){Serial.print(F("> Fallo al conectar, intentando otra vez en "));Serial.print(MODBUS_RECONNECT_TIME/1000);Serial.println(" segundos");}
        dseErrorComm[d]=true;//SI NO SE LOGRA LA CONEXION SE ACTIVA EL ERROR DE CONEXION DEL DSE ACTUAL
      } else {
        if(debug){Serial.println(F("> Conectado a servidor Modbus"));}
        dseErrorComm[d]=false;//SI SE LOGRA LA CONEXION SE BORRA EL ERROR DE CONEXION DEL DSE ACTUAL
      }
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
      if(debug){
        Serial.print(F("> Cliente Modbus conectado: ")); //a new client connected
        Serial.print(newClient.remoteIP());
        Serial.print(" , Total:");
        Serial.println(client_cnt);
      }
      break;
      }
    }
  }
  //If there are packets available, receive them and process them.
  for (byte i = 0; i < NUMBER_OF_MODBUS_CLIENTS; i++) {
    if (clients[i].available()) { //if data is available
      modbusTCPServer.accept(clients[i]); //accept that data
      modbusTCPServer.poll();// service any Modbus TCP requests, while client connected
      readModbusCoils();
      readModbusHoldingRegisters();
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

void readDseAlarms(){//esta funcion lee los registros de alarma de los DSE

  for(int d=0;d<NUMBER_OF_DSE;d++){
    if(dseErrorComm[d]){//SI HAY UN ERROR DE CONEXION CON EL DSE ACTUAL SE SIGUE CON EL SIGUIENTE
      limpiarAlarma(d);//SE LIMPIAN LAS ALARMAS DEL MODULO QUE TIENE UN ERROR DE CONEXION
      continue;
    }

    if (!modbusTCPClient[d].connected()) {// client not connected, start the Modbus TCP client
      if(debug){
        Serial.print(F("> Intentando conectar al servidor Modbus con IP:"));
        Serial.println(servers[d]);
      }

      if (!modbusTCPClient[d].begin(servers[d])) {
        if(debug){Serial.print(F("> Fallo al conectar, intentando otra vez en "));Serial.print(MODBUS_RECONNECT_TIME/1000);Serial.println(" segundos");}
        dseErrorComm[d]=true;//SI NO SE LOGRA LA CONEXION SE ACTIVA EL ERROR DE CONEXION DEL DSE ACTUAL
      } else {
        if(debug){Serial.println(F("> Conectado a servidor Modbus"));}
        dseErrorComm[d]=false;//SI SE LOGRA LA CONEXION SE BORRA EL ERROR DE CONEXION DEL DSE ACTUAL
      }
    } else {
      if(modbusTCPClient[d].requestFrom(HOLDING_REGISTERS,39425,37)){//LEYENDO LOS REGITROS DE ALARM
        if(modbusTCPClient[d].available()){
          for(int i=0;i<37;i++){
            dseIR[d][i] = modbusTCPClient[d].read();//COPIANDO LAS ALARMAS EN EL ARRAY CORRESPONDIENTE
          }
        }
      }else{
        Serial.println(modbusTCPClient[d].lastError());
        dseErrorComm[d]=true;//SE ACTIVA EL ERROR SI SE DETECTA UN ERROR EN LA LECTURA DE LOS REGISTROS
      }
    }
  }
}

void computeDseAlarms(){
  //esta funcion separa las 4 alarmas que vienen dentro del mismo registro
  //cada alarma es de 4 bits y solo se busca que este entre los valores de 2 a 4
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      //SEPARANDO
      int a1 = dseIR[i][j]&0b00001111;
      int a2 = (dseIR[i][j]>>4)&0b00001111;
      int a3 = (dseIR[i][j]>>8)&0b00001111;
      int a4 = (dseIR[i][j]>>12)&0b00001111;
      //DETECTANDO SI EL VALOR ESTA ENTRE 2 Y 4
      dseAlarms[i][j*4]=a1<=4 && a1>=2;
      dseAlarms[i][(j*4)+1]=a2<=4 && a2>=2;
      dseAlarms[i][(j*4)+2]=a3<=4 && a3>=2;
      dseAlarms[i][(j*4)+3]=a4<=4 && a4>=2;
    }
  }
}

void utilidades(){
  //updating frame
  frame = micros() - beforeFrame;
  beforeFrame = micros();
  //printing frame time and memory usage
  if(frameEvent.run() && debugUtilidades){
    Serial.print("> Frame time(us): ");
    Serial.println(frame);
    printMemory();
    Serial.print("> DSE comm error: ");
    for(int i=0; i<NUMBER_OF_DSE;i++){
      Serial.print(dseErrorComm[i]);
      Serial.print(" ");
    }
    // Serial.print("Schedule coils: ");
    // for(int i=0; i<18;i++){
    //   Serial.print(schCoils[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
    // Serial.print("Schedule holding: ");
    // for(int i=0; i<5;i++){
    //   Serial.print(schHolding[i]);
    //   Serial.print(" ");
    // }
    Serial.println("\n");

  }

}

void printMemory(){
  Serial.print(F("> MEM FREE: "));
  Serial.print(freeMemory(), DEC);
  Serial.print(", ");
  float percent = freeMemory();
  percent = percent*100/32000;
  Serial.print(percent,2);
  Serial.println("%");
}

void test(){
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      dseIR[i][j]=12657;
    }
  }
}

void limpiarAlarma(int modulo){
  for(int j=0;j<150;j++){
    dseAlarms[modulo][j]=0;
  }
}

void readModuleDate(){
  if(!dseErrorComm[0] && modbusTCPClient[0].connected()){
    if(modbusTCPClient[0].requestFrom(HOLDING_REGISTERS,1792,2)){
      if(modbusTCPClient[0].available()){
        unsigned long time = modbusTCPClient[0].read() << 16 | modbusTCPClient[0].read();
        rtc.setEpoch(time);
        rtcUpdate.setFrecuency(UPDATE_DATE_PERIOD);
        Serial.print(F("> Actualizando RTC\n\t"));
        Serial.println(getTime());
        Serial.print("\t");
        Serial.println(getDate());
      }
    }
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

void computeSchRegisters(){
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
    schDurationTimer.setFrecuency(2000);
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
void readModbusCoils(){
  updateModulesDates = modbusTCPServer.coilRead(0);

  gen1Breaker = modbusTCPServer.coilRead(11);
  gen2Breaker = modbusTCPServer.coilRead(12);
  gen3Breaker = modbusTCPServer.coilRead(13);
  gen4Breaker = modbusTCPServer.coilRead(14);
  master1BusAvailable = modbusTCPServer.coilRead(15);
  master2BusAvailable = modbusTCPServer.coilRead(16);
  master3BusAvailable = modbusTCPServer.coilRead(17);
  master4BusAvailable = modbusTCPServer.coilRead(18);

  gen1CommonAlarm = modbusTCPServer.coilRead(21);
  gen2CommonAlarm = modbusTCPServer.coilRead(22);
  gen3CommonAlarm = modbusTCPServer.coilRead(23);
  gen4CommonAlarm = modbusTCPServer.coilRead(24);
  master1CommonAlarm = modbusTCPServer.coilRead(25);
  master2CommonAlarm = modbusTCPServer.coilRead(26);
  master3CommonAlarm = modbusTCPServer.coilRead(27);
  master4CommonAlarm = modbusTCPServer.coilRead(28);

  //leyendo los coils del schedule
  for(int i=0; i<18;i++){
    schCoils[i] = modbusTCPServer.coilRead(i+30);
  }
}

void writeModbusCoils(){
  modbusTCPServer.coilWrite(0,updateModulesDates);
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

void writeModbusDiscreteInputs() {
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<150;j++){
      modbusTCPServer.discreteInputWrite((i*150)+j,dseAlarms[i][j]);
    }
  }
}

void writeModbusInputRegisters() {
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
      modbusTCPServer.inputRegisterWrite((i*37)+j,dseIR[i][j]);
    }
  }
}

void readModbusHoldingRegisters(){
  //leyendo registros del schedule
  for(int i=0;i<5;i++){
    schHolding[i]= modbusTCPServer.holdingRegisterRead(i+10);
  }
}

void writeModbusHoldingRegisters(){
  //escribiendo registros del schedule
  for(int i=0;i<5;i++){
    modbusTCPServer.holdingRegisterWrite(i+10,schHolding[i]);
  }
}
