bool SD_Begin(void){//FUNCION DE INICIALIZACION DE MICRO SD
  Serial.print(F("> Inicializando SD card..."));
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Fallo SD card, o no presente"));
    return(0);
  }
  else {
    Serial.println(F("SD card inicializada"));
    return(1);
  }
}

void dataloggerInit(){//FUNCION DE INICIALIZACION DEL DATALOGGER

  Serial.println(F("> Datalogger Init"));

  //////////////////////////////////////////////////////
  //CREACION DE DIRECTORIO
  if(!SD.exists(F("ALARMS"))){//SI EL DIRECTORIO NO EXISTE SE CREA
    Serial.println(F("Console > Directory \"ALARMS\" not found"));
    Serial.println(F("Console > Creating \"ALARMS\" directory"));
    SD.mkdir(F("ALARMS"));
  }
  //////////////////////////////////////////////////////
  //CREACION DE ARCHIVO DE EVENT LOG
  String filename = "ALARMS/" +  String(rtc.getMonth()) + "_" + String(rtc.getYear()) + ".csv";//EL ARCHIVO DE EVENTO LOG SE NOMBRA DE ACUERDO A LA FECHA ACTUAL

  if(!SD.exists(filename)){//SI EL ARCHIVO NO EXISTE, SE CREA Y SE AGREGA LOS TITULOS DE LAS COLUMNAS
    File dat = SD.open(filename,FILE_WRITE);
    if(dat){
      Serial.println(F("> Creando archivo de event log"));
      dat.println(F("DATE,TIME,DESCRIPTION "));
    }
    else {
      Serial.print(F("error opening "));
      Serial.println(filename);
      SD_Begin();
    }
    dat.close();
  }

  //////////////////////////////////////////////////////
  //CREACION DE ARCHIVO DE ALARMAS ACTIVAS
  filename = "ACTIVE.csv";//NOMBRE DEL ARCHIVO DE ALARMAS ACTIVAS

  if(!SD.exists(filename)){//SI EL ARCHIVO NO EXISTE SE CREA Y SE AGREGA LOS TITULOS DE LAS COLUMNAS
    File dat = SD.open(filename,FILE_WRITE);
    if(dat){
      Serial.println(F("> Creando archivo de alarmas"));
      dat.println(F("DATE,TIME,DESCRIPTION "));
    }
    else {
      Serial.print(F("error opening "));
      Serial.println(filename);
      SD_Begin();
    }
    dat.close();
  }
}

void datalogger(){//FUNCION PARA EL DATALOGGER DEL EVENT LOG

  String filename = "ALARMS/" +  String(rtc.getMonth()) + "_" + String(rtc.getYear()) + ".csv";//EL ARCHIVO SE BUSCA DE ACUERDO AL MES Y AÑO ENVIADO POR ARGUMENTO A LA FUNCION

  if(!SD.exists(filename)){//SI NO EXISTE EL ARCHIVO SE REINICIA EL DATALOGGER
    dataloggerInit();
  }

  File dat = SD.open(filename,FILE_WRITE);

  if(dat){//SI SE ABRE EL ARCHIVO SIN ERROR
    //se debe ir a la posicion 23 "dat.seek(23)" para leer la primera linea
    //cada linea consecutiva debera ser 23 +(55*linea)
    //////////////////////////////////////////////////////
    // guardando fecha
    dat.print(twoDigits(rtc.getMonth()));
    dat.print("/");
    dat.print(twoDigits(rtc.getDay()));
    dat.print("/");
    dat.print(twoDigits(rtc.getYear()));
    dat.print(",");
    //////////////////////////////////////////////////////
    //guardando hora
    dat.print(twoDigits(rtc.getHours()));
    dat.print(":");
    dat.print(twoDigits(rtc.getMinutes()));
    dat.print(":");
    dat.print(twoDigits(rtc.getSeconds()));
    dat.print(",");
    //////////////////////////////////////////////////////
    //GUARDANDO EL TEXTO DEL EVENTO
    dat.print(dataWriteSD);
    //////////////////////////////////////////////////////
    //RELLENANDO PARA COMPLETAR EL ESPACIO DE LINEA
    unsigned int lineaActual = (dat.size()-23)/55;
    unsigned int faltante = 23 + ((lineaActual+1)*55) - dat.size();
    for (int i=0;i<faltante-1;i++){
      dat.print(" ");
    }
    //////////////////////////////////////////////////////
    dat.println("");
    dat.close();
  }
  else {//SI NO SE ABRE EL ARCHIVO SE REINICIA LA MICRO SD
    Serial.print("error opening ");
    Serial.println(filename);
    SD_Begin();
  }
}

void dataloggerRead(int _month, int _year){//FUNCION PARA ESCRIBIR POR MODBUS EL EVENT LOG
  String filename = "ALARMS/" +  String(_month) + "_" + String(_year) + ".csv";//EL ARCHIVO SE BUSCA DE ACUERDO AL MES Y AÑO ENVIADO POR ARGUMENTO A LA FUNCION

  if(!SD.exists(filename)){//SI NO EXISTE EL ARCHIVO SE REINICIA EL DATALOGGER
    dataloggerInit();
  }

  int index = 0;//INDEX DEL ARRAY UTILIZADO PARA GUARDAR EL TEXTO
  int linea = 0;//LINEA QUE SE ESTA LEYENDO
  int cantidadLineas = 0;//CANTIDAD DE LINEAS DEL ARCHIVO
  File dat = SD.open(filename,FILE_READ);

  if(dat){//SI SE ABRIO EL ARCHIVO SIN ERROR
    for(int i=800;i<1036;i++){//LIMPIANDO TODOS LOS DATOS DE LOS REGISTROS
      modbusTCPServer.holdingRegisterWrite(i,0);
    }

    linea = 10*tabla;//LA LINEA INICIAL SE CALCULA DE ACUERDO A LA TABLA QUE SE QUIERE MOSTRAR
    cantidadLineas = (dat.size() - 24)/55;//CALCULO DE TOTAL DE LINEAS

    while(linea<cantidadLineas){//ASEGURANDO QUE NO SE LEA UNA LINEA QUE NO EXISTA

      dat.seek(23+1+(55*linea));//MOVIENDO HACIA LA LINEA QUE SE DESEA MOSTRAR
      index=0;

      for(int i=0;i<55;i++){//ESCRIBIENDO LOS DATOS EN EL ARRAY
        char letra = dat.read();
        alarmLine[index] = letra;
        index++;
      }

      writeAlarmsLineModbus(linea-(tabla*10),800);//LLAMANDO LA FUNCION PARA UNA LINEA COMPLETA EN LOS REGISTROS MODBUS
      linea ++;

      if(linea >= (tabla+1)*10){//SE SALE DEL BUQULE SI LA LINEA SE PASA DE LA TABLA A MOSTRAR
        break;
      }

    }

    dat.close();

  } else {//SI NO SE PUDO ABRIR EL ARCHIVO
    Serial.print("error opening ");
    Serial.println(filename);
    for (int i=800;i<1240;i++){//LIMPIANDO LOS REGISTROS MODBUS
      modbusTCPServer.holdingRegisterWrite(i,0);
    }
    SD_Begin();//REINICIANDO LA MICRO SD
  }

}

void alarmsLogger(){//FUNCION PARA EL DATALOGGER DE LAS ALARMAS ACTIVAS

  if(!SD.exists(F("ACTIVE.csv"))){//SI EL ARCHIVO NO EXISTE SE REINICIA EL DATALOGGER
    dataloggerInit();
  }

  File dat = SD.open(F("ACTIVE.csv"),FILE_WRITE);

  if(dat){//SI SE ABRE EL ARCHIVO SI ERROR
    //se debe ir a la posicion 23 "dat.seek(23)" para leer la primera linea
    //cada linea consecutiva debera ser 23 +(55*linea)
    //////////////////////////////////////////////////////
    // guardando fecha
    dat.print(twoDigits(rtc.getMonth()));
    dat.print("/");
    dat.print(twoDigits(rtc.getDay()));
    dat.print("/");
    dat.print(twoDigits(rtc.getYear()));
    dat.print(",");
    //////////////////////////////////////////////////////
    //guardando hora
    dat.print(twoDigits(rtc.getHours()));
    dat.print(":");
    dat.print(twoDigits(rtc.getMinutes()));
    dat.print(":");
    dat.print(twoDigits(rtc.getSeconds()));
    dat.print(",");
    //////////////////////////////////////////////////////
    dat.print(dataWriteSD);//GUARDANDO EL TEXTO DE LA ALARMA
    //////////////////////////////////////////////////////
    //RELLENANDO PARA COMPLETAR EL ESPACIO DE LINEA
    unsigned int lineaActual = (dat.size()-23)/55;
    unsigned int faltante = 23 + ((lineaActual+1)*55) - dat.size();
    for (int i=0;i<faltante-1;i++){
      dat.print(" ");
    }
    //////////////////////////////////////////////////////
    dat.println("");
    dat.close();
  }
  else {//SI NO SE ABRE EL ARCHIVO SE REINICIA LA MICRO SD
    Serial.print("error opening ");
    Serial.println(F("ACTIVE.csv"));
    SD_Begin();
  }
}

void alarmsLoggerRead(){//FUNCION PARA ESCRIBIR POR MODBUS LAS ALARMAS ACTIVAS

  String filename = F("ACTIVE.csv");

  if(!SD.exists(filename)){//SI EL ARCHIVO NO EXISTE SE REINICIA EL DATALOGGER
    dataloggerInit();
  }

  int index = 0;//INDEX DEL ARRAY UTILIZADO PARA GUARDAR EL TEXTO
  int linea = 0;//LINEA QUE SE ESTA LEYENDO
  int cantidadLineas = 0;//CANTIDAD DE LINEAS DEL ARCHIVO
  File dat = SD.open(filename,FILE_READ);

   if(dat){//SI SE ABRIO EL ARCHIVO SIN ERROR
    for(int i=1300;i<1536;i++){//LIMPIANDO TODOS LOS DATOS DE LOS REGISTROS
      modbusTCPServer.holdingRegisterWrite(i,0);
    }

    linea = 10*tablaActive;//LA LINEA INICIAL SE CALCULA DE ACUERDO A LA TABLA QUE SE QUIERE MOSTRAR
    cantidadLineas = (dat.size() - 24)/55;//CALCULO DE TOTAL DE LINEAS

    while(linea<cantidadLineas){//ASEGURANDO QUE NO SE LEA UNA LINEA QUE NO EXISTA

      dat.seek(23+1+(55*linea));//MOVIENDO HACIA LA LINEA QUE SE DESEA MOSTRAR
      index=0;

      for(int i=0;i<55;i++){//ESCRIBIENDO LOS DATOS EN EL ARRAY
        char letra = dat.read();
        alarmLine[index] = letra;
        index++;
      }

      writeAlarmsLineModbus(linea-(tablaActive*10),1300);//LLAMANDO LA FUNCION PARA UNA LINEA COMPLETA EN LOS REGISTROS MODBUS
      linea ++;

      if(linea >= (tablaActive+1)*10){//SE SALE DEL BUQULE SI LA LINEA SE PASA DE LA TABLA A MOSTRAR
        break;
      }

    }

    dat.close();

  } else {//SI NO SE PUDO ABRIR EL ARCHIVO
    Serial.print("error opening ");
    Serial.println(filename);
    for (int i=1300;i<1560;i++){//LIMPIANDO LOS REGISTROS MODBUS
      modbusTCPServer.holdingRegisterWrite(i,0);
    }
    SD_Begin();//REINICIANDO LA MICRO SD
  }

}
