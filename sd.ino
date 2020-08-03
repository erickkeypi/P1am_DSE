void dataloggerRead(int _month, int _year){
  String filename = "ALARMS/" +  String(_month) + "_" + String(_year) + ".csv";
  if(!SD.exists(filename)){
    dataloggerInit();
  }
  int index = 0;
  int linea = 0;
  int inicioData = 0;
  bool inicioLeido = false;
  File dat = SD.open(filename,FILE_READ);
   if(dat){
    while(dat.available()){
      char letra = dat.read();
      if(inicioData){
        alarmLine[index] = letra;
        index++;
        if(letra == '\n'){
          writeAlarmsLineModbus(linea);
          index = 0;
          linea++;
        }
      }
      if(!inicioLeido && letra == '\n'){
        inicioData = dat.position()+1;
        inicioLeido =true;
        Serial.println(inicioData);
      }

    }
    dat.close();
  }
  else {
    Serial.print("error opening ");
    Serial.println(filename);
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
      dat.println(F("DATE,TIME,DESCRIPTION"));
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
    //guardando fecha
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
    dat.println(dataWriteSD);
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
