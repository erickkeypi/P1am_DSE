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
    switch (modoLectura){
      case READ_MASTER_AND_GEN:
      break;

      case READ_ONLY_GEN:
      if(d==0 || d==2 || d==3|| d==4){
        dseErrorComm[d]=true;
        continue;
      }
      break;

      case READ_ONLY_MASTER:
      if(d==1 || d==5 || d==6|| d==7){
        dseErrorComm[d]=true;
        continue;
      }
      break;
    }
    if (!modbusTCPClient[d].connected()) {// client not connected, start the Modbus TCP client
      Serial.print(F("> Intentando conectar al servidor Modbus con IP:"));
      Serial.println(servers[d]);
      if (!modbusTCPClient[d].begin(servers[d])) {
        Serial.print(F("> Fallo al conectar, intentando otra vez en "));
        Serial.print(MODBUS_RECONNECT_TIME/1000);Serial.println(" segundos");
        dseErrorComm[d]=true;//SI NO SE LOGRA LA CONEXION SE ACTIVA EL ERROR DE CONEXION DEL DSE ACTUAL
      } else {
        Serial.println(F("> Conectado a servidor Modbus"));
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

void readDse(){//esta funcion lee los registros de alarma de los DSE
  for(int d=0;d<NUMBER_OF_DSE;d++){
    if(dseErrorComm[d]){//SI HAY UN ERROR DE CONEXION CON EL DSE ACTUAL SE SIGUE CON EL SIGUIENTE
      limpiarAlarma(d);//SE LIMPIAN LAS ALARMAS DEL MODULO QUE TIENE UN ERROR DE CONEXION
      for(int j=0;j<20;j++){//limpiando las variables principales
        variablesPrincipales[d][j] = 0;
      }
      variablesPrincipales[d][9] = 'N'<<8 | 'o';
      variablesPrincipales[d][10] = ' '<<8 | 'C';
      variablesPrincipales[d][11] = 'o'<<8 | 'n';
      variablesPrincipales[d][12] = 'e'<<8 | 'c';
      variablesPrincipales[d][13] = 't'<<8 | ' ';
      variablesPrincipales[d][14] = 0;
      continue;
    }

    if (!modbusTCPClient[d].connected()) {// client not connected, start the Modbus TCP client
      connectModules();
      // if(debug){
      //   Serial.print(F("> Intentando conectar al servidor Modbus con IP:"));
      //   Serial.println(servers[d]);
      // }
      //
      // if (!modbusTCPClient[d].begin(servers[d])) {
      //   if(debug){Serial.print(F("> Fallo al conectar, intentando otra vez en "));Serial.print(MODBUS_RECONNECT_TIME/1000);Serial.println(" segundos");}
      //   dseErrorComm[d]=true;//SI NO SE LOGRA LA CONEXION SE ACTIVA EL ERROR DE CONEXION DEL DSE ACTUAL
      // } else {
      //   if(debug){Serial.println(F("> Conectado a servidor Modbus"));}
      //   dseErrorComm[d]=false;//SI SE LOGRA LA CONEXION SE BORRA EL ERROR DE CONEXION DEL DSE ACTUAL
      // }
    } else {
      if(modbusTCPClient[d].requestFrom(HOLDING_REGISTERS,39425,37)){//LEYENDO LOS REGITROS DE ALARM
        if(modbusTCPClient[d].available()){
          for(int i=0;i<37;i++){
            dseIR[d][i] = modbusTCPClient[d].read();//COPIANDO LAS ALARMAS EN EL ARRAY CORRESPONDIENTE
          }
        }
      }
      else {
        Serial.println(modbusTCPClient[d].lastError());
        dseErrorComm[d]=true;//SE ACTIVA EL ERROR SI SE DETECTA UN ERROR EN LA LECTURA DE LOS REGISTROS
      }
      if(!dseErrorComm[d]){
      //LEYENDO VARIABLES PARA LA PANTALLA PRINCIPAL
        if(d == 0 || (d >= 2 && d <= 4)){//si es master
          variablesPrincipales[d][0] = modbusTCPClient[d].holdingRegisterRead(1059);//HZ
          variablesPrincipales[d][1] = modbusTCPClient[d].holdingRegisterRead(1699);//V(1)
          variablesPrincipales[d][2] = modbusTCPClient[d].holdingRegisterRead(1698);//V(2)
          variablesPrincipales[d][3] = modbusTCPClient[d].holdingRegisterRead(1561);//KW(1)
          variablesPrincipales[d][4] = modbusTCPClient[d].holdingRegisterRead(1560);//KW(2)
          variablesPrincipales[d][5] = modbusTCPClient[d].holdingRegisterRead(1582);//%
          variablesPrincipales[d][6] = modbusTCPClient[d].holdingRegisterRead(1577);//KVar(1)
          variablesPrincipales[d][7] = modbusTCPClient[d].holdingRegisterRead(1576);//KVar(2)

          dseInputs[d][0] = modbusTCPClient[d].holdingRegisterRead(48658);//MAINS AVAILABLE
          dseInputs[d][1] = modbusTCPClient[d].holdingRegisterRead(48661);//BUS AVAILABLE
          dseInputs[d][2] = modbusTCPClient[d].holdingRegisterRead(48659);//MAIN BRK
          dseInputs[d][3] = modbusTCPClient[d].holdingRegisterRead(48660);//BUS BRK
          dseInputs[d][4] = dseInputs[d][2] | dseInputs[d][3];

        } else if(d == 1 || (d > 4)){//si es generador
          variablesPrincipales[d][0] = modbusTCPClient[d].holdingRegisterRead(1031);//HZ
          variablesPrincipales[d][1] = modbusTCPClient[d].holdingRegisterRead(1651);//V(1)
          variablesPrincipales[d][2] = modbusTCPClient[d].holdingRegisterRead(1650);//V(2)
          variablesPrincipales[d][3] = modbusTCPClient[d].holdingRegisterRead(1537);//KW(1)
          variablesPrincipales[d][4] = modbusTCPClient[d].holdingRegisterRead(1536);//KW(2)
          variablesPrincipales[d][5] = modbusTCPClient[d].holdingRegisterRead(1558);//%
          variablesPrincipales[d][6] = modbusTCPClient[d].holdingRegisterRead(1553);//KVar(1)
          variablesPrincipales[d][7] = modbusTCPClient[d].holdingRegisterRead(1552);//KVar(2)

          dseInputs[d][1] = modbusTCPClient[d].holdingRegisterRead(48661);//GEN AVAILABLE
          dseInputs[d][2] = modbusTCPClient[d].holdingRegisterRead(48659);//GEN BRK
        }
        variablesPrincipales[d][8] = modbusTCPClient[d].holdingRegisterRead(772);//MODE
        for(int j=0;j<6;j++){//LEYENDO EL NOMBRE
          variablesPrincipales[d][9+(j)] = nombres[d][j*2]<<8 | nombres[d][(j*2)+1];
        }
      }
    }
  }

  //master screen
  if(dseErrorComm[masterActual]){//SI HAY UN ERROR DE CONEXION
    for(int i=0;i<60;i++){//borrando los valores
      masterScreen[i]=0;
    }
    masterScreen[52] = 'N'<<8 | 'o';
    masterScreen[53] = ' '<<8 | 'C';
    masterScreen[54] = 'o'<<8 | 'n';
    masterScreen[55] = 'e'<<8 | 'c';
    masterScreen[56] = 't'<<8 | 'a';
    masterScreen[57] = 'd'<<8 | 'o';
  }else{
    masterScreen[0] = dseInputs[masterActual][0];//mains available
    masterScreen[1] = dseInputs[masterActual][1];//bus available
    masterScreen[2] = dseInputs[masterActual][2];//mains brk
    masterScreen[3] = dseInputs[masterActual][3];//bus brk
    masterScreen[4] = dseInputs[masterActual][4];//load on
    masterScreen[5] = modbusTCPClient[masterActual].holdingRegisterRead(1699);//maisn l-l avr
    masterScreen[6] = modbusTCPClient[masterActual].holdingRegisterRead(1698);//maisn l-l avr
    masterScreen[7] = variablesPrincipales[masterActual][0];//mains hz
    masterScreen[8] = variablesPrincipales[masterActual][3];//mains kw
    masterScreen[9] = variablesPrincipales[masterActual][4];//mains kw
    masterScreen[10] = variablesPrincipales[masterActual][5];//mains %
    masterScreen[11] = modbusTCPClient[masterActual].holdingRegisterRead(1715);//bus l-l avr
    masterScreen[12] = modbusTCPClient[masterActual].holdingRegisterRead(1714);//bus l-l avr
    masterScreen[13] = modbusTCPClient[masterActual].holdingRegisterRead(1091);//bus freq
    masterScreen[14] = modbusTCPClient[masterActual].holdingRegisterRead(1606);//bus %
    masterScreen[15] = modbusTCPClient[masterActual].holdingRegisterRead(1061);//mains l1-n
    masterScreen[16] = modbusTCPClient[masterActual].holdingRegisterRead(1060);//mains l1-n
    masterScreen[17] = modbusTCPClient[masterActual].holdingRegisterRead(1063);//mains l2-n
    masterScreen[18] = modbusTCPClient[masterActual].holdingRegisterRead(1062);//mains l2-n
    masterScreen[19] = modbusTCPClient[masterActual].holdingRegisterRead(1065);//mains l3-n
    masterScreen[20] = modbusTCPClient[masterActual].holdingRegisterRead(1064);//mains l3-n
    masterScreen[21] = modbusTCPClient[masterActual].holdingRegisterRead(1067);//mains l1-l2
    masterScreen[22] = modbusTCPClient[masterActual].holdingRegisterRead(1066);//mains l1-l2
    masterScreen[23] = modbusTCPClient[masterActual].holdingRegisterRead(1069);//mains l2-l3
    masterScreen[24] = modbusTCPClient[masterActual].holdingRegisterRead(1068);//mains l2-l3
    masterScreen[25] = modbusTCPClient[masterActual].holdingRegisterRead(1071);//mains l3-l4
    masterScreen[26] = modbusTCPClient[masterActual].holdingRegisterRead(1070);//mains l3-l4
    masterScreen[27] = modbusTCPClient[masterActual].holdingRegisterRead(1077);//current l1
    masterScreen[28] = modbusTCPClient[masterActual].holdingRegisterRead(1076);//current l1
    masterScreen[29] = modbusTCPClient[masterActual].holdingRegisterRead(1079);//current l2
    masterScreen[30] = modbusTCPClient[masterActual].holdingRegisterRead(1078);//current l2
    masterScreen[31] = modbusTCPClient[masterActual].holdingRegisterRead(1081);//current l3
    masterScreen[32] = modbusTCPClient[masterActual].holdingRegisterRead(1080);//current l3
    masterScreen[33] = modbusTCPClient[masterActual].holdingRegisterRead(1093);//bus l1-n
    masterScreen[34] = modbusTCPClient[masterActual].holdingRegisterRead(1092);//bus l1-n
    masterScreen[35] = modbusTCPClient[masterActual].holdingRegisterRead(1095);//bus l2-n
    masterScreen[36] = modbusTCPClient[masterActual].holdingRegisterRead(1094);//bus l2-n
    masterScreen[37] = modbusTCPClient[masterActual].holdingRegisterRead(1097);//bus l3-n
    masterScreen[38] = modbusTCPClient[masterActual].holdingRegisterRead(1096);//bus l3-n
    masterScreen[39] = modbusTCPClient[masterActual].holdingRegisterRead(1099);//bus l1-l2
    masterScreen[40] = modbusTCPClient[masterActual].holdingRegisterRead(1098);//bus l1-l2
    masterScreen[41] = modbusTCPClient[masterActual].holdingRegisterRead(1101);//bus l2-l3
    masterScreen[42] = modbusTCPClient[masterActual].holdingRegisterRead(1100);//bus l2-l3
    masterScreen[43] = modbusTCPClient[masterActual].holdingRegisterRead(1103);//bus l3-l1
    masterScreen[44] = modbusTCPClient[masterActual].holdingRegisterRead(1102);//bus l3-l1
    masterScreen[45] = modbusTCPClient[masterActual].holdingRegisterRead(1581);//pf
    masterScreen[46] = modbusTCPClient[masterActual].holdingRegisterRead(1569);//mains kva
    masterScreen[47] = modbusTCPClient[masterActual].holdingRegisterRead(1568);//mains kva
    masterScreen[48] = modbusTCPClient[masterActual].holdingRegisterRead(1577);//mains KVar
    masterScreen[49] = modbusTCPClient[masterActual].holdingRegisterRead(1576);//mains kvar
    masterScreen[50] = modbusTCPClient[masterActual].holdingRegisterRead(1074);//phase rotation
    masterScreen[51] = variablesPrincipales[masterActual][8];//modo

    for(int j=0;j<6;j++){//LEYENDO EL NOMBRE
      masterScreen[52+j] = variablesPrincipales[masterActual][9+j];
    }

    if(masterButtonPress){
      if(modbusTCPClient[masterActual].beginTransmission(HOLDING_REGISTERS,4104,2)){
        modbusTCPClient[masterActual].write(masterScreen[59]);
        modbusTCPClient[masterActual].write(masterScreen[58]);
        modbusTCPClient[masterActual].endTransmission();
      } else{
        dseErrorComm[masterActual]=true;
      }
      masterButtonPress = false;
      Serial.print(F("> Master "));
      Serial.print(masterActual);
      Serial.println(" system key pressed");
    }
  }
  //generador screen
  if(dseErrorComm[genActual]){//SI HAY UN ERROR DE CONEXION
    for(int i=0;i<60;i++){//borrando los valores
      genScreen[i]=0;
    }
    genScreen[48] = 'N'<<8 | 'o';
    genScreen[49] = ' '<<8 | 'C';
    genScreen[50] = 'o'<<8 | 'n';
    genScreen[51] = 'e'<<8 | 'c';
    genScreen[52] = 't'<<8 | 'a';
    genScreen[53] = 'd'<<8 | 'o';
  } else{
    genScreen[0] = dseInputs[genActual][1];//gen available
    genScreen[1] = dseInputs[genActual][2];//gen brk
    genScreen[2] = modbusTCPClient[genActual].holdingRegisterRead(1659);//gen l-l avr
    genScreen[3] = modbusTCPClient[genActual].holdingRegisterRead(1658);//gen l-l avr
    genScreen[4] = variablesPrincipales[genActual][0];//gen hz
    genScreen[5] = variablesPrincipales[genActual][3];//gen kw
    genScreen[6] = variablesPrincipales[genActual][4];//gen kw
    genScreen[7] = variablesPrincipales[genActual][5];//gen %
    genScreen[8] = modbusTCPClient[genActual].holdingRegisterRead(1715);//bus l-l avr
    genScreen[9] = modbusTCPClient[genActual].holdingRegisterRead(1714);//bus l-l avr
    genScreen[10] = modbusTCPClient[genActual].holdingRegisterRead(1091);//bus freq
    genScreen[11] = modbusTCPClient[genActual].holdingRegisterRead(1606);//bus %
    genScreen[12] = modbusTCPClient[genActual].holdingRegisterRead(1033);//gen l1-n
    genScreen[13] = modbusTCPClient[genActual].holdingRegisterRead(1032);//gen l1-n
    genScreen[14] = modbusTCPClient[genActual].holdingRegisterRead(1035);//gen l2-n
    genScreen[15] = modbusTCPClient[genActual].holdingRegisterRead(1034);//gen l2-n
    genScreen[16] = modbusTCPClient[genActual].holdingRegisterRead(1037);//gen l3-n
    genScreen[17] = modbusTCPClient[genActual].holdingRegisterRead(1036);//gen l3-n
    genScreen[18] = modbusTCPClient[genActual].holdingRegisterRead(1039);//gen l1-l2
    genScreen[19] = modbusTCPClient[genActual].holdingRegisterRead(1038);//gen l1-l2
    genScreen[20] = modbusTCPClient[genActual].holdingRegisterRead(1041);//gen l2-l3
    genScreen[21] = modbusTCPClient[genActual].holdingRegisterRead(1040);//gen l2-l3
    genScreen[22] = modbusTCPClient[genActual].holdingRegisterRead(1043);//gen l3-l1
    genScreen[23] = modbusTCPClient[genActual].holdingRegisterRead(1042);//gen l3-l1
    genScreen[24] = modbusTCPClient[genActual].holdingRegisterRead(1045);//gen l3-l1
    genScreen[25] = modbusTCPClient[genActual].holdingRegisterRead(1044);//gen l3-l1
    genScreen[26] = modbusTCPClient[genActual].holdingRegisterRead(1047);//gen l3-l1
    genScreen[27] = modbusTCPClient[genActual].holdingRegisterRead(1046);//gen l3-l1
    genScreen[28] = modbusTCPClient[genActual].holdingRegisterRead(1049);//gen l3-l1
    genScreen[29] = modbusTCPClient[genActual].holdingRegisterRead(1048);//gen l3-l1
    genScreen[30] = modbusTCPClient[genActual].holdingRegisterRead(1074);//pf
    genScreen[31] = modbusTCPClient[genActual].holdingRegisterRead(1545);//kva
    genScreen[32] = modbusTCPClient[genActual].holdingRegisterRead(1544);//kva
    genScreen[33] = modbusTCPClient[genActual].holdingRegisterRead(1553);//kvar
    genScreen[34] = modbusTCPClient[genActual].holdingRegisterRead(1552);//kvar
    genScreen[35] = modbusTCPClient[genActual].holdingRegisterRead(1073);//fase roteacion
    genScreen[36] = modbusTCPClient[genActual].holdingRegisterRead(1024);//oil press
    genScreen[37] = modbusTCPClient[genActual].holdingRegisterRead(1029);//battery
    genScreen[38] = modbusTCPClient[genActual].holdingRegisterRead(1030);//engine speed
    genScreen[39] = modbusTCPClient[genActual].holdingRegisterRead(1025);//coolant temp
    genScreen[40] = modbusTCPClient[genActual].holdingRegisterRead(1027);//fuel level
    genScreen[41] = modbusTCPClient[genActual].holdingRegisterRead(1799);//engine runtime
    genScreen[42] = modbusTCPClient[genActual].holdingRegisterRead(1798);//engine runtime
    genScreen[43] = modbusTCPClient[genActual].holdingRegisterRead(1809);//number starts
    genScreen[44] = modbusTCPClient[genActual].holdingRegisterRead(1808);//number starts
    genScreen[45] = modbusTCPClient[genActual].holdingRegisterRead(1801);//kwh
    genScreen[46] = modbusTCPClient[genActual].holdingRegisterRead(1800);//kwh
    genScreen[47] = variablesPrincipales[genActual][8];//modo

    for(int j=0;j<6;j++){//LEYENDO EL NOMBRE
      genScreen[48+j] = variablesPrincipales[genActual][9+j];
    }

    if(genButtonPress){
      if(modbusTCPClient[genActual].beginTransmission(HOLDING_REGISTERS,4104,2)){
        modbusTCPClient[genActual].write(genScreen[55]);
        modbusTCPClient[genActual].write(genScreen[54]);
        modbusTCPClient[genActual].endTransmission();
      } else{
        dseErrorComm[genActual]=true;
      }
      genButtonPress = false;
      Serial.println("> Gen system key pressed");
    }


  }

  //bus screen el bus solo sale delos gen
  if(modoLectura == READ_ONLY_GEN|| modoLectura == READ_MASTER_AND_GEN){
    // for(int i=0;i<50;i++){
    //   busScreen[i]=0;
    // }
    unsigned long currentL1=0;
    unsigned long currentL2=0;
    unsigned long currentL3=0;
    unsigned long totalWattsL1=0;
    unsigned long totalWattsL2=0;
    unsigned long totalWattsL3=0;
    for(int i=0;i<NUMBER_OF_DSE;i++){//leyendo el total de corriente y watts
      if(i==0 || (i>=2 && i<=4)){//saltando los masters
        continue;
      }
      if(!dseErrorComm[i]){
        currentL1 += modbusTCPClient[i].holdingRegisterRead(1105) << 16 | modbusTCPClient[i].holdingRegisterRead(1104);//gen1 bus current l1
        currentL2 += modbusTCPClient[i].holdingRegisterRead(1107) << 16 | modbusTCPClient[i].holdingRegisterRead(1106);//gen1 bus current l2
        currentL3 += modbusTCPClient[i].holdingRegisterRead(1109) << 16 | modbusTCPClient[i].holdingRegisterRead(1108);//gen1 bus current l3

        totalWattsL1 += modbusTCPClient[i].holdingRegisterRead(1113) << 16 | modbusTCPClient[i].holdingRegisterRead(1112);//gen1 bus total watts l1
        totalWattsL2 += modbusTCPClient[i].holdingRegisterRead(1115) << 16 | modbusTCPClient[i].holdingRegisterRead(1114);//gen1 bus total watts l2
        totalWattsL3 += modbusTCPClient[i].holdingRegisterRead(1117) << 16 | modbusTCPClient[i].holdingRegisterRead(1116);//gen1 bus total watts l3

      }
    }

    busScreen[19] = currentL1;
    busScreen[20] = currentL1 >> 16;
    busScreen[21] = currentL2;
    busScreen[22] = currentL2 >> 16;
    busScreen[23] = currentL3;
    busScreen[24] = currentL3 >> 16;
    busScreen[25] = totalWattsL1;
    busScreen[26] = totalWattsL1 >> 16;
    busScreen[27] = totalWattsL2;
    busScreen[28] = totalWattsL2 >> 16;
    busScreen[29] = totalWattsL3;
    busScreen[30] = totalWattsL3 >> 16;

    if(!dseErrorComm[1]){
      busScreen[33] = modbusTCPClient[1].holdingRegisterRead(35104);//priority
      busScreen[37] = modbusTCPClient[1].holdingRegisterRead(30);//quality
    }
    if(!dseErrorComm[5]){
      busScreen[34] = modbusTCPClient[5].holdingRegisterRead(35104);//priority
      busScreen[38] = modbusTCPClient[5].holdingRegisterRead(30);//quality
    }
    if(!dseErrorComm[6]){
      busScreen[35] = modbusTCPClient[6].holdingRegisterRead(35104);//priority
      busScreen[39] = modbusTCPClient[6].holdingRegisterRead(30);//quality
    }

    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(i==0 || (i>=2 && i<=4)){//saltando los masters
        continue;
      }
      if(!dseErrorComm[i]){//si no hay error se leen los valores del bus una sola vez (break)
        busScreen[0] = modbusTCPClient[i].holdingRegisterRead(1585);// bus total watts
        busScreen[1] = modbusTCPClient[i].holdingRegisterRead(1584);// bus total watts
        busScreen[2] = modbusTCPClient[i].holdingRegisterRead(1715);// bus l-l
        busScreen[3] = modbusTCPClient[i].holdingRegisterRead(1714);// bus l-l
        busScreen[4] = modbusTCPClient[i].holdingRegisterRead(1707);// bus l-n
        busScreen[5] = modbusTCPClient[i].holdingRegisterRead(1706);// bus l-n
        busScreen[6] = modbusTCPClient[i].holdingRegisterRead(1606);// bus %
        busScreen[7] = modbusTCPClient[i].holdingRegisterRead(1093);// bus l1-n
        busScreen[8] = modbusTCPClient[i].holdingRegisterRead(1092);// bus l1-n
        busScreen[9] = modbusTCPClient[i].holdingRegisterRead(1095);// bus l2-n
        busScreen[10] = modbusTCPClient[i].holdingRegisterRead(1094);// bus l2-n
        busScreen[11] = modbusTCPClient[i].holdingRegisterRead(1097);// bus l3-n
        busScreen[12] = modbusTCPClient[i].holdingRegisterRead(1096);// bus l3-n
        busScreen[13] = modbusTCPClient[i].holdingRegisterRead(1099);// bus l1-l2
        busScreen[14] = modbusTCPClient[i].holdingRegisterRead(1098);// bus l1-l2
        busScreen[15] = modbusTCPClient[i].holdingRegisterRead(1101);// bus l2-l3
        busScreen[16] = modbusTCPClient[i].holdingRegisterRead(1100);// bus l2-l3
        busScreen[17] = modbusTCPClient[i].holdingRegisterRead(1103);// bus l3-l1
        busScreen[18] = modbusTCPClient[i].holdingRegisterRead(1102);// bus l3-l1

        busScreen[31] = modbusTCPClient[i].holdingRegisterRead(1091);// bus freq
        busScreen[32] = modbusTCPClient[i].holdingRegisterRead(1118);// bus fase rot

        busScreen[41] = modbusTCPClient[i].holdingRegisterRead(29);// masters online
        busScreen[42] = modbusTCPClient[i].holdingRegisterRead(28);// gens online
        break;
      }
    }
    for(int i=0;i<50;i++){//escribiendo los registros para la pantalla de generador
      modbusTCPServer.holdingRegisterWrite(700+i,busScreen[i]);
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
    modoLecturaCallback();
    Serial.print(F("> Frame time(us): "));
    Serial.println(frame);
    printMemory();
    Serial.print(F("> DSE comm error: "));
    for(int i=0; i<NUMBER_OF_DSE;i++){
      Serial.print(dseErrorComm[i]);
      Serial.print(F(" "));
    }
    Serial.println();

    Serial.println();
    // modbusTCPClient[1].holdingRegisterWrite(35104,2);//priority
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

  switch (modoLectura) {
    case READ_MASTER_AND_GEN:
    dseBase =0;
    break;

    case READ_ONLY_GEN:
    dseBase =1;
    break;

    case READ_ONLY_MASTER:
    dseBase =0;
    break;
  }

  if(!dseErrorComm[dseBase] && modbusTCPClient[dseBase].connected()){
    if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){
      if(modbusTCPClient[dseBase].available()){
        unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();
        rtc.setEpoch(time);
        rtcUpdate.setFrecuency(UPDATE_DATE_PERIOD);
        Serial.print(F("> Actualizando RTC\n\t"));
        Serial.println(getTime());
        Serial.print("\t");
        Serial.println(getDate());
        modbusTCPServer.holdingRegisterWrite(1240,rtc.getMonth());
        modbusTCPServer.holdingRegisterWrite(1241,rtc.getYear());
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

void updateDseDates(){
  switch (modoLectura) {
    case READ_MASTER_AND_GEN:
    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(i == dseBase){//saltando el dse base
        continue;
      }
      if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){
        if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){
          if(modbusTCPClient[dseBase].available()){
            unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();

            if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){
              modbusTCPClient[i].write(time >> 16);
              modbusTCPClient[i].write(time);
              modbusTCPClient[i].endTransmission();
              Serial.println(F("> RTC de modulo actualizado"));
            }
          }
        }
      } else {
        Serial.println(F("> Error al actualizar RTC de modulo"));
      }
    }
    break;

    case READ_ONLY_GEN:
    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(i == dseBase || i==0 || i==2 || i==3 || i==4){//saltando el dse y los master
        continue;
      }
      if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){
        if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){
          if(modbusTCPClient[dseBase].available()){
            unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();

            if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){
              modbusTCPClient[i].write(time >> 16);
              modbusTCPClient[i].write(time);
              modbusTCPClient[i].endTransmission();
              Serial.println(F("> RTC de modulo actualizado"));
            }
          }
        }
      } else {
        Serial.println(F("> Error al actualizar RTC de modulo"));
      }
    }
    break;

    case READ_ONLY_MASTER:
    for(int i=0;i<NUMBER_OF_DSE;i++){
      if(i == dseBase || i==1 || i==5 || i==6 || i==7){//saltando el dse y los gens
        continue;
      }
      if(!dseErrorComm[i] && modbusTCPClient[dseBase].connected()){
        if(modbusTCPClient[dseBase].requestFrom(HOLDING_REGISTERS,1792,2)){
          if(modbusTCPClient[dseBase].available()){
            unsigned long time = modbusTCPClient[dseBase].read() << 16 | modbusTCPClient[dseBase].read();

            if(modbusTCPClient[i].beginTransmission(HOLDING_REGISTERS,1792,2)){
              modbusTCPClient[i].write(time >> 16);
              modbusTCPClient[i].write(time);
              modbusTCPClient[i].endTransmission();
              Serial.println(F("> RTC de modulo actualizado"));
            }
          }
        }
      } else {
        Serial.println(F("> Error al actualizar RTC de modulo"));
      }
    }
    break;
  }
}

////MODBUS
void readModbusCoils(){
  updateModulesDates = modbusTCPServer.coilRead(0);
  masterButtonPress = modbusTCPServer.coilRead(1);
  genButtonPress = modbusTCPServer.coilRead(2);

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

void writeModbusDiscreteInputs() {
  for(int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<10;j++){
      modbusTCPServer.discreteInputWrite((i*10)+j,dseInputs[i][j]);
    }
  }
  for(int i=0;i<NUMBER_OF_DSE;i++){
    modbusTCPServer.discreteInputWrite(100+i,dseErrorComm[i]);
  }
}

void writeModbusInputRegisters() {
  for (int i=0;i<NUMBER_OF_DSE;i++){
    for(int j=0;j<37;j++){
    //modbusTCPServer.inputRegisterWrite((i*37)+j,dseIR[i][j]);
    }
  }
}

void readModbusHoldingRegisters(){
  //leyendo registros del schedule
  for(int i=0;i<5;i++){
    schHolding[i]= modbusTCPServer.holdingRegisterRead(i+10);
  }
  masterActual = modbusTCPServer.holdingRegisterRead(499);
  masterScreen[58]= modbusTCPServer.holdingRegisterRead(558);
  masterScreen[59]= modbusTCPServer.holdingRegisterRead(559);
  genActual = modbusTCPServer.holdingRegisterRead(599);
  genScreen[54]= modbusTCPServer.holdingRegisterRead(654);
  genScreen[55]= modbusTCPServer.holdingRegisterRead(655);
  if(masterActual != 0 && masterActual != 2 && masterActual != 3 && masterActual != 4){
    masterActual=0;
  }
  if(genActual != 1 && genActual != 5 && genActual !=6){
    genActual=1;
  }

  if(modoLectura == READ_MASTER_AND_GEN || modoLectura == READ_ONLY_GEN){
    if(!dseErrorComm[1]){//cambiando prioridad gen 1
      busScreen[33] = constrain(modbusTCPServer.holdingRegisterRead(733),1,32);
      if(modbusTCPClient[1].holdingRegisterRead(35104) != busScreen[33]){
        if(modbusTCPClient[1].beginTransmission(HOLDING_REGISTERS,35104,1)){
          modbusTCPClient[1].write(busScreen[33]);
          modbusTCPClient[1].endTransmission();
        }
      }
    }
    if(!dseErrorComm[5]){//cambiando prioridad gen 2
      busScreen[34] = constrain(modbusTCPServer.holdingRegisterRead(734),1,32);
      if(modbusTCPClient[5].holdingRegisterRead(35104) != busScreen[34]){
        if(modbusTCPClient[5].beginTransmission(HOLDING_REGISTERS,35104,1)){
          modbusTCPClient[5].write(busScreen[34]);
          modbusTCPClient[5].endTransmission();
        }
      }
    }
    if(!dseErrorComm[6]){//cambiando prioridad gen 3
      busScreen[35] = constrain(modbusTCPServer.holdingRegisterRead(735),1,32);
      if(modbusTCPClient[6].holdingRegisterRead(35104) != busScreen[35]){
        if(modbusTCPClient[6].beginTransmission(HOLDING_REGISTERS,35104,1)){
          modbusTCPClient[6].write(busScreen[35]);
          modbusTCPClient[6].endTransmission();
        }
      }
    }
  }
}

void writeModbusHoldingRegisters(){
  //escribiendo registros del schedule
  for(int i=0;i<5;i++){
    modbusTCPServer.holdingRegisterWrite(i+10,schHolding[i]);
  }

  for(int i=0;i<NUMBER_OF_DSE;i++){
    for (int j =0;j<20;j++){
      modbusTCPServer.holdingRegisterWrite(100+(i*20)+j,variablesPrincipales[i][j]);
    }
  }

  for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de master
    modbusTCPServer.holdingRegisterWrite(500+i,masterScreen[i]);
  }

  for(int i=0;i<60;i++){//escribiendo los registros para la pantalla de generador
    modbusTCPServer.holdingRegisterWrite(600+i,genScreen[i]);
  }

  for(int i=0;i<50;i++){//escribiendo los registros para la pantalla de generador
    modbusTCPServer.holdingRegisterWrite(700+i,busScreen[i]);
  }
  // modbusTCPClient[1].holdingRegisterWrite(35104,2);//priority
  unsigned int monthServer = modbusTCPServer.holdingRegisterRead(1240);
  unsigned int yearServer = modbusTCPServer.holdingRegisterRead(1241);
  if(monthServer>12){
    monthServer =1;
    yearServer++;
  }
  if(monthServer<1){
    monthServer =12;
    yearServer--;
  }

  modbusTCPServer.holdingRegisterWrite(1240,monthServer);
  modbusTCPServer.holdingRegisterWrite(1241,yearServer);

}

void writeAlarmsLineModbus(unsigned int _reg){
  int primeraComa = 0;
  int segundaComa = 0;
  int final =0;
  for(int i=0;i<50;i++){
    if(alarmLine[i] == ','){
      primeraComa = i;
      break;
    }
  }
  for(int i=primeraComa+1;i<50;i++){
    if(alarmLine[i] == ','){
      segundaComa = i;
      break;
    }
  }
  for(int i=segundaComa+1;i<50;i++){
    if(alarmLine[i] == '\n'){
      final = i;
      break;
    }else{
      final =49;
    }
  }

  char buff[30];
  for (int i=0;i<30;i++){
    buff[i]=0;
  }
  for(int i=0;i<primeraComa;i++){
    buff[i] = alarmLine[i];
  }
  for(int j=0;j<5;j++){//
    modbusTCPServer.holdingRegisterWrite(800+j+(_reg*5),buff[j*2] <<8 | buff[j*2+1]);
  }

  for (int i=0;i<30;i++){
    buff[i]=0;
  }
  for(int i=primeraComa+1;i<segundaComa;i++){
    buff[i-(primeraComa+1)] = alarmLine[i];
  }
  for(int j=0;j<5;j++){//
    modbusTCPServer.holdingRegisterWrite(850+j+(_reg*5),buff[j*2] <<8 | buff[j*2+1]);
  }

  for (int i=0;i<30;i++){
    buff[i]=0;
  }
  for(int i=segundaComa+1;i<final;i++){
    buff[i-(segundaComa+1)] = alarmLine[i];
  }
  for(int j=0;j<15;j++){//
    modbusTCPServer.holdingRegisterWrite(900+j+(_reg*15),buff[j*2] <<8 | buff[j*2+1]);
  }


}
