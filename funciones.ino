void initializeArrays(){//FUNCION QUE INICIALIZA LOS ARRAYS
//   if(debug){Serial.println(F("> Arrays inicializados"));}
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       for(int j=0;j<150;j++){
//         dseAlarms[i][j]=0;
//       }
//     }
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       for(int j=0;j<150;j++){
//         oldDseAlarms[i][j]=0;
//       }
//     }
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       for(int j=0;j<37;j++){
//         dseIR[i][j]=false;
//       }
//     }
    for(int i=0; i<8; i++){
      dseErrorComm[i]=false;
    }
//     for(int i=0; i<8; i++){
//       oldDseErrorComm[i]=false;
//     }
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
      break;

      case READ_ONLY_GEN:
      if(modulos[d].model == DSE_8660MKII){//
        dseErrorComm[d]=true;
        continue;
      }
      break;

      case READ_ONLY_MASTER:
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
//   for(int d=0;d<NUMBER_OF_DSE;d++){
//     //////////////////////////////////////////////////////
//     //DETERMINANDO EL ERROR DE CONEXION
//     if(dseErrorComm[d]){//SI HAY UN ERROR DE CONEXION CON EL DSE ACTUAL SE SIGUE CON EL SIGUIENTE
//       limpiarAlarma(d);//SE LIMPIAN LAS ALARMAS DEL MODULO QUE TIENE UN ERROR DE CONEXION
//     }
//     //////////////////////////////////////////////////////
//     //LEYENDO LOS REGISTROS DE ALARMAS
//       //////////////////////////////////////////////////////
//       //LEYENDO VARIABLES PARA LA PANTALLA PRINCIPAL
//     }
//   }
//   //////////////////////////////////////////////////////
//   //PANTALLA DE MASTER
//   if(dseErrorComm[masterActual]){//SI HAY UN ERROR DE CONEXION
//   }else{
//
//     if(masterButtonPress){//SI SE PRESIONA UN BOTON DE COMANDO ESTE SE ENVIA AL MODULO DSE
//       if(modbusTCPClient[masterActual].beginTransmission(HOLDING_REGISTERS,4104,2)){
//         modbusTCPClient[masterActual].write(masterScreen[59]);
//         modbusTCPClient[masterActual].write(masterScreen[58]);
//         modbusTCPClient[masterActual].endTransmission();
//       } else{
//         dseErrorComm[masterActual]=true;
//       }
//       masterButtonPress = false;
//       Serial.print(F("> Master "));
//       Serial.print(masterActual);
//       Serial.println(F(" system key pressed"));
//     }
//   }
//
//   //////////////////////////////////////////////////////
//   //PANTALLA DE GENERADOR
//   if(dseErrorComm[genActual]){//SI HAY UN ERROR DE CONEXION
//     for(int i=0;i<60;i++){//borrando los valores
//       genScreen[i]=0;
//     }
//     genScreen[48] = 'N'<<8 | 'o';
//     genScreen[49] = ' '<<8 | 'C';
//     genScreen[50] = 'o'<<8 | 'n';
//     genScreen[51] = 'e'<<8 | 'c';
//     genScreen[52] = 't'<<8 | 'a';
//     genScreen[53] = 'd'<<8 | 'o';
//   } else{
//     genScreen[0] = dseInputs[genActual][1];//gen available
//     genScreen[1] = dseInputs[genActual][2];//gen brk
//     genScreen[2] = modbusTCPClient[genActual].holdingRegisterRead(1659);//gen l-l avr
//     genScreen[3] = modbusTCPClient[genActual].holdingRegisterRead(1658);//gen l-l avr
//     genScreen[4] = variablesPrincipales[genActual][0];//gen hz
//     genScreen[5] = variablesPrincipales[genActual][3];//gen kw
//     genScreen[6] = variablesPrincipales[genActual][4];//gen kw
//     genScreen[7] = variablesPrincipales[genActual][5];//gen %
//     genScreen[8] = modbusTCPClient[genActual].holdingRegisterRead(1715);//bus l-l avr
//     genScreen[9] = modbusTCPClient[genActual].holdingRegisterRead(1714);//bus l-l avr
//     genScreen[10] = modbusTCPClient[genActual].holdingRegisterRead(1091);//bus freq
//     genScreen[11] = modbusTCPClient[genActual].holdingRegisterRead(1606);//bus %
//     genScreen[12] = modbusTCPClient[genActual].holdingRegisterRead(1033);//gen l1-n
//     genScreen[13] = modbusTCPClient[genActual].holdingRegisterRead(1032);//gen l1-n
//     genScreen[14] = modbusTCPClient[genActual].holdingRegisterRead(1035);//gen l2-n
//     genScreen[15] = modbusTCPClient[genActual].holdingRegisterRead(1034);//gen l2-n
//     genScreen[16] = modbusTCPClient[genActual].holdingRegisterRead(1037);//gen l3-n
//     genScreen[17] = modbusTCPClient[genActual].holdingRegisterRead(1036);//gen l3-n
//     genScreen[18] = modbusTCPClient[genActual].holdingRegisterRead(1039);//gen l1-l2
//     genScreen[19] = modbusTCPClient[genActual].holdingRegisterRead(1038);//gen l1-l2
//     genScreen[20] = modbusTCPClient[genActual].holdingRegisterRead(1041);//gen l2-l3
//     genScreen[21] = modbusTCPClient[genActual].holdingRegisterRead(1040);//gen l2-l3
//     genScreen[22] = modbusTCPClient[genActual].holdingRegisterRead(1043);//gen l3-l1
//     genScreen[23] = modbusTCPClient[genActual].holdingRegisterRead(1042);//gen l3-l1
//     genScreen[24] = modbusTCPClient[genActual].holdingRegisterRead(1045);//gen l3-l1
//     genScreen[25] = modbusTCPClient[genActual].holdingRegisterRead(1044);//gen l3-l1
//     genScreen[26] = modbusTCPClient[genActual].holdingRegisterRead(1047);//gen l3-l1
//     genScreen[27] = modbusTCPClient[genActual].holdingRegisterRead(1046);//gen l3-l1
//     genScreen[28] = modbusTCPClient[genActual].holdingRegisterRead(1049);//gen l3-l1
//     genScreen[29] = modbusTCPClient[genActual].holdingRegisterRead(1048);//gen l3-l1
//     genScreen[30] = modbusTCPClient[genActual].holdingRegisterRead(1557);//pf
//     genScreen[31] = modbusTCPClient[genActual].holdingRegisterRead(1545);//kva
//     genScreen[32] = modbusTCPClient[genActual].holdingRegisterRead(1544);//kva
//     genScreen[33] = modbusTCPClient[genActual].holdingRegisterRead(1553);//kvar
//     genScreen[34] = modbusTCPClient[genActual].holdingRegisterRead(1552);//kvar
//     genScreen[35] = modbusTCPClient[genActual].holdingRegisterRead(1073);//fase roteacion
//     genScreen[36] = modbusTCPClient[genActual].holdingRegisterRead(1024);//oil press
//     genScreen[37] = modbusTCPClient[genActual].holdingRegisterRead(1029);//battery
//     genScreen[38] = modbusTCPClient[genActual].holdingRegisterRead(1030);//engine speed
//     genScreen[39] = modbusTCPClient[genActual].holdingRegisterRead(1025);//coolant temp
//     genScreen[40] = modbusTCPClient[genActual].holdingRegisterRead(1027);//fuel level
//     genScreen[41] = modbusTCPClient[genActual].holdingRegisterRead(1799);//engine runtime
//     genScreen[42] = modbusTCPClient[genActual].holdingRegisterRead(1798);//engine runtime
//     genScreen[43] = modbusTCPClient[genActual].holdingRegisterRead(1809);//number starts
//     genScreen[44] = modbusTCPClient[genActual].holdingRegisterRead(1808);//number starts
//     genScreen[45] = modbusTCPClient[genActual].holdingRegisterRead(1801);//kwh
//     genScreen[46] = modbusTCPClient[genActual].holdingRegisterRead(1800);//kwh
//     genScreen[47] = variablesPrincipales[genActual][8];//modo
//
//     for(int j=0;j<6;j++){//LEYENDO EL NOMBRE
//       genScreen[48+j] = variablesPrincipales[genActual][9+j];
//     }
//
//     if(genButtonPress){//SI SE PRESIONA UN BOTON DE COMANDO ESTE SE ENVIA AL MODULO DSE
//       if(modbusTCPClient[genActual].beginTransmission(HOLDING_REGISTERS,4104,2)){
//         modbusTCPClient[genActual].write(genScreen[55]);
//         modbusTCPClient[genActual].write(genScreen[54]);
//         modbusTCPClient[genActual].endTransmission();
//       } else{
//         dseErrorComm[genActual]=true;
//       }
//       genButtonPress = false;
//       Serial.print(F("> Gen "));
//       Serial.print(genActual);
//       Serial.println(" system key pressed");
//     }
//   }
//
//   //////////////////////////////////////////////////////
//   //PANTALLA DE BUS. SOLO SALE DE LOS GEN
//   if(modoLectura == READ_ONLY_GEN|| modoLectura == READ_MASTER_AND_GEN){
//     // for(int i=0;i<50;i++){
//     //   busScreen[i]=0;
//     // }
//     unsigned long currentL1=0;
//     unsigned long currentL2=0;
//     unsigned long currentL3=0;
//     unsigned long totalWattsL1=0;
//     unsigned long totalWattsL2=0;
//     unsigned long totalWattsL3=0;
//
//     //LEYENDO Y SUMANDO LA CORRIENTE Y LOS WATTS DE LOS GEN
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       if(i==0 || (i>=2 && i<=4)){//saltando los masters
//         continue;
//       }
//
//       if(!dseErrorComm[i]){
//         currentL1 += modbusTCPClient[i].holdingRegisterRead(1105) << 16 | modbusTCPClient[i].holdingRegisterRead(1104);//gen1 bus current l1
//         currentL2 += modbusTCPClient[i].holdingRegisterRead(1107) << 16 | modbusTCPClient[i].holdingRegisterRead(1106);//gen1 bus current l2
//         currentL3 += modbusTCPClient[i].holdingRegisterRead(1109) << 16 | modbusTCPClient[i].holdingRegisterRead(1108);//gen1 bus current l3
//
//         totalWattsL1 += modbusTCPClient[i].holdingRegisterRead(1113) << 16 | modbusTCPClient[i].holdingRegisterRead(1112);//gen1 bus total watts l1
//         totalWattsL2 += modbusTCPClient[i].holdingRegisterRead(1115) << 16 | modbusTCPClient[i].holdingRegisterRead(1114);//gen1 bus total watts l2
//         totalWattsL3 += modbusTCPClient[i].holdingRegisterRead(1117) << 16 | modbusTCPClient[i].holdingRegisterRead(1116);//gen1 bus total watts l3
//       }
//     }
//
//     //GUARDANDO LOS VALORES SUMADOS EN EL ARRAY
//     busScreen[19] = currentL1;
//     busScreen[20] = currentL1 >> 16;
//     busScreen[21] = currentL2;
//     busScreen[22] = currentL2 >> 16;
//     busScreen[23] = currentL3;
//     busScreen[24] = currentL3 >> 16;
//     busScreen[25] = totalWattsL1;
//     busScreen[26] = totalWattsL1 >> 16;
//     busScreen[27] = totalWattsL2;
//     busScreen[28] = totalWattsL2 >> 16;
//     busScreen[29] = totalWattsL3;
//     busScreen[30] = totalWattsL3 >> 16;
//
//     //PRIORIDAD Y QUALITY
//     if(!dseErrorComm[1]){
//       busScreen[33] = modbusTCPClient[1].holdingRegisterRead(35104);//priority
//       busScreen[37] = modbusTCPClient[1].holdingRegisterRead(30);//quality
//     }
//     if(!dseErrorComm[5]){
//       busScreen[34] = modbusTCPClient[5].holdingRegisterRead(35104);//priority
//       busScreen[38] = modbusTCPClient[5].holdingRegisterRead(30);//quality
//     }
//     if(!dseErrorComm[6]){
//       busScreen[35] = modbusTCPClient[6].holdingRegisterRead(35104);//priority
//       busScreen[39] = modbusTCPClient[6].holdingRegisterRead(30);//quality
//     }
//
//     //LEYENDO OTROS VALORES
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       if(i==0 || (i>=2 && i<=4)){//saltando los masters
//         continue;
//       }
//       if(!dseErrorComm[i]){//si no hay error se leen los valores del bus una sola vez (break)
//         busScreen[0] = modbusTCPClient[i].holdingRegisterRead(1585);// bus total watts
//         busScreen[1] = modbusTCPClient[i].holdingRegisterRead(1584);// bus total watts
//         busScreen[2] = modbusTCPClient[i].holdingRegisterRead(1715);// bus l-l
//         busScreen[3] = modbusTCPClient[i].holdingRegisterRead(1714);// bus l-l
//         busScreen[4] = modbusTCPClient[i].holdingRegisterRead(1707);// bus l-n
//         busScreen[5] = modbusTCPClient[i].holdingRegisterRead(1706);// bus l-n
//         busScreen[6] = modbusTCPClient[i].holdingRegisterRead(1606);// bus %
//         busScreen[7] = modbusTCPClient[i].holdingRegisterRead(1093);// bus l1-n
//         busScreen[8] = modbusTCPClient[i].holdingRegisterRead(1092);// bus l1-n
//         busScreen[9] = modbusTCPClient[i].holdingRegisterRead(1095);// bus l2-n
//         busScreen[10] = modbusTCPClient[i].holdingRegisterRead(1094);// bus l2-n
//         busScreen[11] = modbusTCPClient[i].holdingRegisterRead(1097);// bus l3-n
//         busScreen[12] = modbusTCPClient[i].holdingRegisterRead(1096);// bus l3-n
//         busScreen[13] = modbusTCPClient[i].holdingRegisterRead(1099);// bus l1-l2
//         busScreen[14] = modbusTCPClient[i].holdingRegisterRead(1098);// bus l1-l2
//         busScreen[15] = modbusTCPClient[i].holdingRegisterRead(1101);// bus l2-l3
//         busScreen[16] = modbusTCPClient[i].holdingRegisterRead(1100);// bus l2-l3
//         busScreen[17] = modbusTCPClient[i].holdingRegisterRead(1103);// bus l3-l1
//         busScreen[18] = modbusTCPClient[i].holdingRegisterRead(1102);// bus l3-l1
//
//         busScreen[31] = modbusTCPClient[i].holdingRegisterRead(1091);// bus freq
//         busScreen[32] = modbusTCPClient[i].holdingRegisterRead(1118);// bus fase rot
//
//         busScreen[41] = modbusTCPClient[i].holdingRegisterRead(29);// masters online
//         busScreen[42] = modbusTCPClient[i].holdingRegisterRead(28);// gens online
//         break;
//       }
//     }
//     for(int i=0;i<50;i++){//escribiendo los registros para la pantalla de generador
//       modbusTCPServer.holdingRegisterWrite(700+i,busScreen[i]);
//     }
//   }
}
//
// void computeDseAlarms(){//esta funcion separa las 4 alarmas que vienen dentro del mismo registro
//   //cada alarma es de 4 bits y solo se busca que este entre los valores de 2 a 4
//   for(int i=0;i<NUMBER_OF_DSE;i++){
//     for(int j=0;j<37;j++){
//       //SEPARANDO
//       int a1 = dseIR[i][j]&0b00001111;
//       int a2 = (dseIR[i][j]>>4)&0b00001111;
//       int a3 = (dseIR[i][j]>>8)&0b00001111;
//       int a4 = (dseIR[i][j]>>12)&0b00001111;
//       //DETECTANDO SI EL VALOR ESTA ENTRE 2 Y 4
//       dseAlarms[i][j*4]=a1<=4 && a1>=2;
//       dseAlarms[i][(j*4)+1]=a2<=4 && a2>=2;
//       dseAlarms[i][(j*4)+2]=a3<=4 && a3>=2;
//       dseAlarms[i][(j*4)+3]=a4<=4 && a4>=2;
//     }
//   }
// }
//
// void limpiarAlarma(int modulo){//FUNCION QUE LIMPIA LAS ALARMAS
//   for(int j=0;j<150;j++){
//     dseAlarms[modulo][j]=0;
//   }
// }
//
//
// void readModuleDate(){//FUNCION QUE LEE LA FECHA DEL MODULO ELEGIDO COMO BASE
//
//   switch (modoLectura) {//EL MODULO BASE CAMBIA DE ACUERDO AL MODO DE LECTURA
//     case READ_MASTER_AND_GEN:
//     dseBase =0;
//     break;
//
//     case READ_ONLY_GEN:
//     dseBase =1;
//     break;
//
//     case READ_ONLY_MASTER:
//     dseBase =0;
//     break;
//   }
//   //LEYENDO LA FECHA DEL MODULO BASE Y ACTUALIZANDO EL RTC
//   if(!dseErrorComm[dseBase] && modbusTCPClient[dseBase].connected()){
//     if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){
//       if(modbusTCPClient[dseBase].available()){
//         unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();
//         rtc.setEpoch(time);
//         rtcUpdate.setFrecuency(UPDATE_DATE_PERIOD);
//         Serial.print(F("> Actualizando RTC\n\t"));
//         Serial.println(getTime());
//         Serial.print("\t");
//         Serial.println(getDate());
//         modbusTCPServer.holdingRegisterWrite(1240,rtc.getMonth());
//         modbusTCPServer.holdingRegisterWrite(1241,rtc.getYear());
//       }
//     }
//   } else {
//     Serial.println(F("> Error al actualizar RTC. Intentando nuevamente en 10 segundos"));
//   }
// }

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


// void updateDseDates(){//FUNCION QUE ACTUALIZA LA FECHA DE LOS DSE TOMANDO UN MODULO BASE
//   switch (modoLectura) {
//     case READ_MASTER_AND_GEN:
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       if(i == dseBase){//saltando el dse base
//         continue;
//       }
//       if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){//REVISANDO SIEL MODULO BASE Y EL ACTUAL ESTAN CONECTADOS
//         if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){//LLEYENDO REGISTROS DE FECHA
//           if(modbusTCPClient[dseBase].available()){
//             unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();
//
//             if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){//ESCRIBIENDO REGISTROS DE FECHA
//               modbusTCPClient[i].write(time >> 16);
//               modbusTCPClient[i].write(time);
//               modbusTCPClient[i].endTransmission();
//               Serial.println(F("> RTC de modulo actualizado"));
//             }
//           }
//         }
//       } else {
//         Serial.println(F("> Error al actualizar RTC de modulo"));
//       }
//     }
//     break;
//
//     case READ_ONLY_GEN:
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       if(i == dseBase || i==0 || i==2 || i==3 || i==4){//saltando el dse BASE y los master
//         continue;
//       }
//       if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){//REVISANDO SIEL MODULO BASE Y EL ACTUAL ESTAN CONECTADOS
//         if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){//LLEYENDO REGISTROS DE FECHA
//           if(modbusTCPClient[dseBase].available()){
//             unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();
//
//             if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){//ESCRIBIENDO REGISTROS DE FECHA
//               modbusTCPClient[i].write(time >> 16);
//               modbusTCPClient[i].write(time);
//               modbusTCPClient[i].endTransmission();
//               Serial.println(F("> RTC de modulo actualizado"));
//             }
//           }
//         }
//       } else {
//         Serial.println(F("> Error al actualizar RTC de modulo"));
//       }
//     }
//     break;
//
//     case READ_ONLY_MASTER:
//     for(int i=0;i<NUMBER_OF_DSE;i++){
//       if(i == dseBase || i==1 || i==5 || i==6 || i==7){//saltando el dse BASE y los gens
//         continue;
//       }
//       if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){//REVISANDO SIEL MODULO BASE Y EL ACTUAL ESTAN CONECTADOS
//         if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){//LLEYENDO REGISTROS DE FECHA
//           if(modbusTCPClient[dseBase].available()){
//             unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();
//
//             if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){//ESCRIBIENDO REGISTROS DE FECHA
//               modbusTCPClient[i].write(time >> 16);
//               modbusTCPClient[i].write(time);
//               modbusTCPClient[i].endTransmission();
//               Serial.println(F("> RTC de modulo actualizado"));
//             }
//           }
//         }
//       } else {
//         Serial.println(F("> Error al actualizar RTC de modulo"));
//       }
//     }
//     break;
//   }
// }

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

// void activateAlarms(){//FUNCION QUE DETERMINA SI ESTA ACTIVA UNA ALARMA Y LA GUARDA
//
//   static bool alarmaDesactivada = false;
//
//   if(alarmaDesactivada){
//     //SI SE DESACTIVA UNA ALARMA SE BORRA EL ARCHIVO DE ALARMAS ACTIVAS
//     //LUEGO SE VUELVE A BUSCAR LAS ALARMAS ACTIVAS Y SE GUARDAN
//     SD.remove(F("ACTIVE.csv"));
//     for(int i=0;i<8;i++){//BUSCANDO ALARMAS DE ERROR DE CONEXION
//       switch (modoLectura) {
//         case READ_ONLY_GEN:
//         if(i==0 || i==2 || i==3 || i==4){//saltando los master
//           continue;
//         }
//         break;
//
//         case READ_ONLY_MASTER:
//         if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//           continue;
//         }
//         break;
//
//         case READ_MASTER_AND_GEN:
//         break;
//       }
//       if(oldDseErrorComm[i]){
//         dataWriteSD = nombres[i];
//         dataWriteSD += F(" COMM ERROR");
//         alarmsLogger();
//       }
//     }
//     for(int i=0;i<NUMBER_OF_DSE;i++){//BUSCANDO ALARMAS DE LOS DSE
//       switch (modoLectura) {
//         case READ_ONLY_GEN:
//         if(i==0 || i==2 || i==3 || i==4){//saltando los master
//           continue;
//         }
//         break;
//
//         case READ_ONLY_MASTER:
//         if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//           continue;
//         }
//         break;
//
//         case READ_MASTER_AND_GEN:
//         break;
//       }
//       for (int j=0;j<150;j++){
//         if(oldDseAlarms[i][j]){
//           dataWriteSD = nombres[i];
//           dataWriteSD += " ";
//           dataWriteSD += DSEAlarmsString[j-1];
//           alarmsLogger();
//         }
//       }
//     }
//     alarmaDesactivada=false;
//   }
//
//   for(int i=0;i<8;i++){//REVISANDO SI SE DESACTIVA UNA ALARMA DE ERROR DE CONEXION
//     switch (modoLectura) {
//       case READ_ONLY_GEN:
//       if(i==0 || i==2 || i==3 || i==4){//saltando los master
//         continue;
//       }
//       break;
//
//       case READ_ONLY_MASTER:
//       if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//         continue;
//       }
//       break;
//
//       case READ_MASTER_AND_GEN:
//       break;
//     }
//     if(oldDseErrorComm[i] && !dseErrorComm[i]){
//       alarmaDesactivada =true;
//       break;
//     }
//   }
//
//   for(int i=0;i<NUMBER_OF_DSE;i++){//REVISANDO SI SE DESACTIVA UNA ALARMA DE UN DSE
//     switch (modoLectura) {
//       case READ_ONLY_GEN:
//       if(i==0 || i==2 || i==3 || i==4){//saltando los master
//         continue;
//       }
//       break;
//
//       case READ_ONLY_MASTER:
//       if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//         continue;
//       }
//       break;
//
//       case READ_MASTER_AND_GEN:
//       break;
//     }
//     for (int j=0;j<150;j++){
//       if(oldDseAlarms[i][j] && !dseAlarms[i][j]){
//         alarmaDesactivada =true;
//         break;
//       }
//       if(alarmaDesactivada){
//         break;
//       }
//     }
//   }
//
//   for(int i=0;i<8;i++){//REVISANDO SI SE ACTIVA UNA ALARMA DE ERROR DE CONEXION
//     switch (modoLectura) {
//       case READ_ONLY_GEN:
//       if(i==0 || i==2 || i==3 || i==4){//saltando los master
//         continue;
//       }
//       break;
//
//       case READ_ONLY_MASTER:
//       if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//         continue;
//       }
//       break;
//
//       case READ_MASTER_AND_GEN:
//       break;
//     }
//     if(!oldDseErrorComm[i] && dseErrorComm[i]){
//       dataWriteSD = nombres[i];
//       dataWriteSD += F(" COMM ERROR");
//       datalogger();//AGREGANDO AL EVENTO LOG
//       alarmsLogger();//AGREGANDO A LAS ALARMAS ACTIVAS
//     }
//     oldDseErrorComm[i] = dseErrorComm[i];
//   }
//
//   for(int i=0;i<NUMBER_OF_DSE;i++){//REVISANDO SI SE ACTIVA UNA ALARMA DE DSE
//     switch (modoLectura) {
//       case READ_ONLY_GEN:
//       if(i==0 || i==2 || i==3 || i==4){//saltando los master
//         continue;
//       }
//       break;
//
//       case READ_ONLY_MASTER:
//       if(i==1 || i==5 || i==6 || i==7){//saltando los gen
//         continue;
//       }
//       break;
//
//       case READ_MASTER_AND_GEN:
//       break;
//     }
//     for (int j=0;j<150;j++){
//       if(!oldDseAlarms[i][j] && dseAlarms[i][j]){
//         dataWriteSD = nombres[i];
//         dataWriteSD += " ";
//         dataWriteSD += DSEAlarmsString[j-1];//SE TOMA EL TEXTO DE LA ALARMA DE DSEAlarms.H
//         datalogger();//AGREGANDO AL EVENT LOG
//         alarmsLogger();//AGREGANDO A LAS ALARMAS ACTIVAS
//       }
//       oldDseAlarms[i][j] = dseAlarms[i][j];
//     }
//   }
// }
//
// void computeSchRegisters(){//FUNCION QUE HACE LOS CALCULOS DEL SCHEDULE
//   //los siguientes "if" hacen que si se disminuye la cantidad menos que el limite inferior esta pasa a su limite superior
//   // si es menor a 0 pasa a 23 y si es mayor a 23 pasa a 0
//   //la forma en que esta implementado es porque la variable es sin signo osea que ninca va a ser menor a 0
//   //cuando se disminuye desde 0 entonces pasa a ser un numero muy grande
//   //por lo que solo basta detectar que sea un numero muy grande
//   if(schHolding[0]>1000){//restringiendo las horas
//     schHour = 23;
//   } else if (schHolding[0]>23){
//     schHour = 0;
//   } else{
//     schHour = schHolding[0];
//   }
//
//   if(schHolding[1]>1000){//restringiendo los minutos
//     schMinute = 59;
//   } else if (schHolding[1]>59){
//     schMinute = 0;
//   }else{
//     schMinute = schHolding[1];
//   }
//
//   if(schHolding[3]<1){//restringiendo los meses
//     schMonth = 12;
//   } else if (schHolding[3]>12){
//     schMonth = 1;
//   }else{
//     schMonth = schHolding[3];
//   }
//
//   if(schHolding[4]>50000){//duracion entre 0 y 30 dias (minutos)
//     schDuration = 43200;
//   } else if (schHolding[4]>43200){
//     schDuration = 0;
//   }else{
//     schDuration = schHolding[4];
//   }
//
//   switch (schMonth){//se restringen los dias dependiendo el mes
//     case 1://enero
//     case 3://marzo
//     case 5://mayo
//     case 7://julio
//     case 8://agosto
//     case 10://octubre
//     case 12://diciembre
//     //se restringen los dias a 31 cuando son los meses correspondientes
//     if(schHolding[2]<1){
//       schDay = 31;
//     } else if (schHolding[2]>31){
//       schDay = 1;
//     }else{
//       schDay = schHolding[2];
//     }
//     break;
//
//     case 2://febrero
//     //se restringe a 28 dias si es febrero
//     if(schHolding[2]<1){
//       schDay = 28;
//     } else if (schHolding[2]>28){
//       schDay = 1;
//     }else{
//       schDay = schHolding[2];
//     }
//     break;
//
//     case 4://abril
//     case 6://junio
//     case 9://septiembre
//     case 11://noviembre
//     //se restringe a 30 dias en los meses correspondientes
//     if(schHolding[2]<1){
//       schDay = 30;
//     } else if (schHolding[2]>30){
//       schDay = 1;
//     }else{
//       schDay = schHolding[2];
//     }
//     break;
//
//     default:
//     //se restringe a 30 dias por defecto
//     if(schHolding[2]<1){
//       schDay = 30;
//     } else if (schHolding[2]>30){
//       schDay = 1;
//     }else{
//       schDay = schHolding[2];
//     }
//     break;
//
//   }
//   //GUARDANDO RESULTADO EN EL ARRAY
//   schHolding[0] = schHour;
//   schHolding[1] = schMinute;
//   schHolding[2] = schDay;
//   schHolding[3] = schMonth;
//   schHolding[4] = schDuration;
//
//   //////////////////////////////////////////////////////
//   //coils
//
//   //enable
//   schEnable = schCoils[7];
//
//   //test off/on load
//   if(schCoils[13] && schTestLoad == SCH_TEST_ON_LOAD){//cambiando a test off load
//     schTestLoad = SCH_TEST_OFF_LOAD;
//     schCoils[14]=false;
//   }
//   if(schCoils[14] && schTestLoad == SCH_TEST_OFF_LOAD){//cambiando a test on load
//     schTestLoad = SCH_TEST_ON_LOAD;
//     schCoils[13]=false;
//   }
//
//   //transition open/closed
//   if(schCoils[15] && schTransition == SCH_TRANSITION_CLOSED){//cambiando a transition open
//     schTransition = SCH_TRANSITION_OPEN;
//     schCoils[16]=false;
//   }
//   if(schCoils[16] && schTransition == SCH_TRANSITION_OPEN){//cambiando a transition closed
//     schTransition = SCH_TRANSITION_CLOSED;
//     schCoils[15]=false;
//   }
//   //load demand Inhibit
//   schLoadDemandInhibit = schCoils[17];
//
//   //tipo repeticion
//   if(schCoils[9] && schTipoRepeticion !=SCH_DAILY){//activar la repeticion diaria y desactivando las demas
//     schTipoRepeticion = SCH_DAILY;
//     schCoils[10] = false;
//     schCoils[11] = false;
//     schCoils[12] = false;
//   }
//   if(schCoils[10] && schTipoRepeticion !=SCH_WEEKLY){//activar la repeticion semanal y desactivando las demas
//     schTipoRepeticion = SCH_WEEKLY;
//     schCoils[9] = false;
//     schCoils[11] = false;
//     schCoils[12] = false;
//   }
//   if(schCoils[11] && schTipoRepeticion !=SCH_MONTHLY){//activar la repeticion mensual y desactivando las demas
//     schTipoRepeticion = SCH_MONTHLY;
//     schCoils[10] = false;
//     schCoils[9] = false;
//     schCoils[12] = false;
//   }
//   if(schCoils[12] && schTipoRepeticion !=SCH_DATE){//activar activacion en fecha y desactivando las demas
//     schTipoRepeticion = SCH_DATE;
//     schCoils[10] = false;
//     schCoils[11] = false;
//     schCoils[9] = false;
//   }
//
//   //computando si se activa el schedule
//   if(schDuration == 0){
//     schDurationTimer.setFrecuency(2000);//TIMER DE 2 SEGUNDOS PARA DESACTIVAR EL SCHEDULE
//   }else{
//     schDurationTimer.setFrecuency(schDuration*60*1000);
//   }
//   switch(schTipoRepeticion){
//
//     case SCH_DAILY://en repeticion diaria solo se comprueba la hora y los minutos
//     if(!schActive && schEnable && (rtc.getHours() == schHour) && (rtc.getMinutes() == schMinute) && (rtc.getSeconds() == 0)){
//       schActive = true;
//       schDurationTimer.start();
//       Serial.print(F("> Schedule activado. Duracion: "));
//       Serial.print(schDuration);
//       Serial.println(F(" minutos"));
//     }
//     break;
//
//     case SCH_WEEKLY:
//     if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0){
//       //0 jueves, 1 viernes, 2 sabado, 3 domingo, 4 lunes, 5 martes, 6 miercoles
//       //rtc.getEpoch();
//       unsigned long dayOfWeek= (rtc.getEpoch() / 86400) % 7;
//       //calculando se el dia corresponde a alguno de los elegidos
//       bool diaActivo = (schCoils[0] && dayOfWeek == 4) || (schCoils[1] && dayOfWeek == 5) || (schCoils[2] && dayOfWeek == 6) || (schCoils[3] && dayOfWeek == 0) || (schCoils[4] && dayOfWeek == 1) || (schCoils[5] && dayOfWeek == 2) || (schCoils[6] && dayOfWeek == 3);
//       if(diaActivo){
//         schActive = true;
//         schDurationTimer.start();
//         Serial.println(F("Schedule activado"));
//       }
//     }
//     break;
//
//     case SCH_MONTHLY:
//     if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0 && rtc.getDay() == schDay){
//       schActive = true;
//       schDurationTimer.start();
//       Serial.println(F("Schedule activado"));
//     }
//     break;
//
//     case SCH_DATE:
//     if(!schActive && schEnable && rtc.getHours() == schHour && rtc.getMinutes() == schMinute && rtc.getSeconds() == 0 && rtc.getDay() == schDay && rtc.getMonth() == schMonth){
//       schActive = true;
//       schDurationTimer.start();
//       Serial.println(F("Schedule activado"));
//     }
//     break;
//   }
//
//   schCoils[8] = schActive;
//
// }
//
//
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

// void writeModbusCoils(){//FUNCION QUE ESCRIBE LOS COILS
//   modbusTCPServer.coilWrite(0,updateModulesDates);
//   modbusTCPServer.coilWrite(1,masterButtonPress);
//   modbusTCPServer.coilWrite(2,genButtonPress);
//   modbusTCPServer.coilWrite(10,busLive);
//   modbusTCPServer.coilWrite(20,generalCommonAlarm);
//   //escribiendo registros del schedule
//   for(int i=0; i<18;i++){
//     modbusTCPServer.coilWrite(i+30,schCoils[i]);
//   }
//   //ESCRIBIENDO LOS ERRORES DE COMUNICACION
//   for(int i=0;i<NUMBER_OF_DSE;i++){
//     modbusTCPServer.coilWrite(i+50,dseErrorComm[i]);
//   }
// }
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
    modbusTCPServer.discreteInputWrite(3+(i*10),modulos[i].busAvailable);
    modbusTCPServer.discreteInputWrite(4+(i*10),modulos[i].busAvailable);
  }
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.discreteInputWrite(100+i,dseErrorComm[i]);
  }
}
//
// void writeModbusInputRegisters() {//FUNCION QUE ESCRIBE LOS INPUTS
//   for (int i=0;i<NUMBER_OF_DSE;i++){
//     for(int j=0;j<37;j++){
//     //modbusTCPServer.inputRegisterWrite((i*37)+j,dseIR[i][j]);
//     }
//   }
// }

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

  // //CAMBIO DE PRIORIDAD
  // if(modoLectura == READ_MASTER_AND_GEN || modoLectura == READ_ONLY_GEN){
  //   if(!dseErrorComm[1]){//cambiando prioridad gen 1
  //     busScreen[33] = constrain(modbusTCPServer.holdingRegisterRead(733),1,32);
  //     if(modbusTCPClient[1].holdingRegisterRead(35104) != busScreen[33]){//SI LA PRIORIDAD ES DIFERENTE ENTONES SE CAMBIA
  //       if(modbusTCPClient[1].beginTransmission(HOLDING_REGISTERS,35104,1)){
  //         modbusTCPClient[1].write(busScreen[33]);
  //         modbusTCPClient[1].endTransmission();
  //       }
  //     }
  //   }
  //   if(!dseErrorComm[5]){//cambiando prioridad gen 2
  //     busScreen[34] = constrain(modbusTCPServer.holdingRegisterRead(734),1,32);
  //     if(modbusTCPClient[5].holdingRegisterRead(35104) != busScreen[34]){//SI LA PRIORIDAD ES DIFERENTE ENTONES SE CAMBIA
  //       if(modbusTCPClient[5].beginTransmission(HOLDING_REGISTERS,35104,1)){
  //         modbusTCPClient[5].write(busScreen[34]);
  //         modbusTCPClient[5].endTransmission();
  //       }
  //     }
  //   }
  //   if(!dseErrorComm[6]){//cambiando prioridad gen 3
  //     busScreen[35] = constrain(modbusTCPServer.holdingRegisterRead(735),1,32);
  //     if(modbusTCPClient[6].holdingRegisterRead(35104) != busScreen[35]){//SI LA PRIORIDAD ES DIFERENTE ENTONES SE CAMBIA
  //       if(modbusTCPClient[6].beginTransmission(HOLDING_REGISTERS,35104,1)){
  //         modbusTCPClient[6].write(busScreen[35]);
  //         modbusTCPClient[6].endTransmission();
  //       }
  //     }
  //   }
  // }
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
//
//   for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de master
//     modbusTCPServer.holdingRegisterWrite(500+i,masterScreen[i]);
//   }
//
//   for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de generador
//     modbusTCPServer.holdingRegisterWrite(600+i,genScreen[i]);
//   }
//
//   for(int i=0;i<50;i++){//escribiendo los registros para la pantalla de generador
//     modbusTCPServer.holdingRegisterWrite(700+i,busScreen[i]);
//   }
//
//   //RESTRINGIENDO EL VALOR DEL MES A MOSTRAR EN EL EVENT LOG
//   unsigned int monthServer = modbusTCPServer.holdingRegisterRead(1240);
//   unsigned int yearServer = modbusTCPServer.holdingRegisterRead(1241);
//   if(monthServer>12){//SI SE AUMENTA EL MES 12 ENTONCES SE SUMA UN AÑO
//     monthServer =1;
//     yearServer++;
//   }
//   if(monthServer<1){//SI SE DISMINUYE EL MES 1 ENTONCES SE RESTA UN AÑO
//     monthServer =12;
//     yearServer--;
//   }
//
//   modbusTCPServer.holdingRegisterWrite(1240,monthServer);
//   modbusTCPServer.holdingRegisterWrite(1241,yearServer);
//
}
//
//
void utilidades(){
  //CALCULANDO EL TIEMPO DE EJECUCION DE UN CICLO DE PROGRAMA (FRAME)
  frame = micros() - beforeFrame;
  beforeFrame = micros();

  if(frameEvent.run() && debugUtilidades){

    modoLecturaCallback();//IMPRIMIENDO EL MODO DE LECTURA

    //IMPRIMIENDO EL TIEMPO DE FRAME
    Serial.print(F("> Frame time(us): "));
    Serial.println(frame);

    //IMPRIMIENDO LA MEMORIA DISPONIBLE
    printMemory();

    Serial.println();
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
