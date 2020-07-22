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
  //sch
  void writeSchCoils();
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
  //sch
  for(int i=0;i<5;i++){
    schHolding[i]= modbusTCPServer.holdingRegisterRead(i+10);
  }
}

void writeModbusHoldingRegisters(){
}

void utilidades(){
  //updating frame
  frame = micros() - beforeFrame;
  beforeFrame = micros();
  //printing frame time and memory usage
  if(frameEvent.run() && debugUtilidades){
    Serial.print("Frame: ");
    Serial.println(frame);
    Serial.print(F("MEM FREE: "));
    Serial.println(freeMemory(), DEC);
    Serial.print("DSE Errors: ");
    for(int i=0; i<NUMBER_OF_DSE;i++){
      Serial.print(dseErrorComm[i]);
      Serial.print(" ");
    }
    Serial.print("Schedule coils: ");
    for(int i=0; i<18;i++){
      Serial.print(schCoils[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("Schedule holding: ");
    for(int i=0; i<5;i++){
      Serial.print(schHolding[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

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

void writeSchCoils(){

}
