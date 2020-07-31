void dataloggerRead(){
  String filename = "ALARMS/" +  String(rtc.getMonth()) + "_" + String(rtc.getYear()) + ".csv";
  if(!SD.exists(filename)){
    dataloggerInit();
  }
  File dat = SD.open(filename,FILE_READ);
   if(dat){
    while(dat.available()){
      Serial.print(char(dat.read()));
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
    dat.print(rtc.getMonth());
    dat.print("/");
    dat.print(rtc.getDay());
    dat.print("/");
    dat.print(rtc.getYear());
    dat.print(",");
    //guardando hora
    dat.print(rtc.getHours());
    dat.print(":");
    dat.print(rtc.getMinutes());
    dat.print(":");
    dat.print(rtc.getSeconds());
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
