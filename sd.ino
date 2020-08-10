void dataloggerRead(int _month, int _year){
  String filename = "ALARMS/" +  String(_month) + "_" + String(_year) + ".csv";
  if(!SD.exists(filename)){
    dataloggerInit();
  }
  int index = 0;
  int linea = 0;
  int cantidadLineas = 0;
  File dat = SD.open(filename,FILE_READ);
   if(dat){
    for(int i=800;i<1036;i++){
      modbusTCPServer.holdingRegisterWrite(i,0);
    }

    linea = 10*tabla;
    cantidadLineas = (dat.size() - 24)/55;
    // Serial.println(cantidadLineas);
    while(linea<cantidadLineas){
      dat.seek(23+1+(55*linea));
      index=0;
      for(int i=0;i<55;i++){
        char letra = dat.read();
        alarmLine[index] = letra;
        index++;
        // Serial.print(letra);
      }
      writeAlarmsLineModbus(linea-(tabla*10));
      linea ++;
      if(linea >= (tabla+1)*10){//si la linea se pasa de la tabla correspondiente
        break;
      }
    }

    dat.close();
  }
  else {
    Serial.print("error opening ");
    Serial.println(filename);
    for (int i=800;i<1240;i++){
      modbusTCPServer.holdingRegisterWrite(i,0);
    }
    SD_Begin();
  }
}

void dataloggerInit(){
  Serial.println(F("> Datalogger Init"));
  if(!SD.exists(F("ALARMS"))){
    Serial.println(F("Console > Directory \"ALARMS\" not found"));
    Serial.println(F("Console > Creating \"ALARMS\" directory"));
    SD.mkdir(F("ALARMS"));
  }
  String filename = "ALARMS/" +  String(rtc.getMonth()) + "_" + String(rtc.getYear()) + ".csv";

  if(!SD.exists(filename)){
    File dat = SD.open(filename,FILE_WRITE);
    if(dat){
      Serial.println(F("> Creando archivo de alarmas"));
      dat.println(F("DATE,TIME,DESCRIPTION "));
      dat.close();
    }
    else {
      Serial.print(F("error opening "));
      Serial.println(filename);
      SD_Begin();
    }
  }
}

void datalogger(){
  String filename = "ALARMS/" +  String(rtc.getMonth()) + "_" + String(rtc.getYear()) + ".csv";
  if(!SD.exists(filename)){
    dataloggerInit();
  }

  File dat = SD.open(filename,FILE_WRITE);
  if(dat){
    //se debe ir a la posicion 23 "dat.seek(23)" para leer la primera linea
    //cada linea consecutiva debera ser 23 +(55*linea)

    // guardando fecha
    dat.print(twoDigits(rtc.getMonth()));
    dat.print("/");
    dat.print(twoDigits(rtc.getDay()));
    dat.print("/");
    dat.print(twoDigits(rtc.getYear()));
    dat.print(",");
    //guardando hora
    dat.print(twoDigits(rtc.getHours()));
    dat.print(":");
    dat.print(twoDigits(rtc.getMinutes()));
    dat.print(":");
    dat.print(twoDigits(rtc.getSeconds()));
    dat.print(",");
    //guardando descipcion de alarma
    dat.print(dataWriteSD);
    //rellenando
    unsigned int lineaActual = (dat.size()-23)/55;
    unsigned int faltante = 23 + ((lineaActual+1)*55) - dat.size();
    for (int i=0;i<faltante-1;i++){
      dat.print(" ");
    }
    dat.println("");
    dat.close();
  }
  else {
    Serial.print("error opening ");
    Serial.println(filename);
    SD_Begin();
  }
}

bool SD_Begin(void){
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
