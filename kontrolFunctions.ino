void updateDateCallback(){
  updateModulesDates=true;
}

void debugCallback(){
  if(debugUtilidades){
    Serial.println(F("> Desactivando debug"));
  }
  else{
    Serial.println(F("> Activando debug"));
  }
  debugUtilidades = !debugUtilidades;
}

void helpCall(){
  Serial.println(F("###HELP###"));
  Serial.println(F("\t{updatedate} -> Actualiza la fecha en los DSE y en el plc"));
  Serial.println(F("\t{debug} -> Activa/desactiva la informacion de debugeo"));
  Serial.println(F("\t{connectmodules} -> Forza la conexion a los DSE"));
  Serial.println(F("\t{printmemory} -> Muestra la cantidad de bytes disponibles en la memoria"));
  Serial.println(F("\t{readmode} -> Muestra el modo de lectura de los DSE"));
  Serial.println(F("##########"));
}

void okCallback(){
  Serial.println(F("> OK"));
}

void modoLecturaCallback(){
  Serial.print(F("> Modo de lectura: "));
  switch (modoLectura) {
    case READ_MASTER_AND_GEN:
    Serial.println(F("READ_MASTER_AND_GEN"));
    break;

    case READ_ONLY_GEN:
    Serial.println(F("READ_ONLY_GEN"));
    break;

    case READ_ONLY_MASTER:
    Serial.println(F("READ_ONLY_MASTER"));
    break;
  }
}

void testCallback(){
  Serial.println(F("> Test callback"));
  muteAllAlarms =true;
  // remoteStartOnLoad(0);
  Serial.println();
}
void test2Callback(){
  Serial.println(F("> Test2 callback"));
  // remoteStartOffLoad(0);
  Serial.println();
}
void test3Callback(){
  Serial.println(F("> Test3 callback"));
  // if(modulos[0].connect()){
  //   Serial.print("Arg: ");
  //   // Serial.println(arg);
  //   modulos[0].beginTransmission(17920,2);//42498//18433//17921
  //   modulos[0].modbusWrite(0x00);
  //   modulos[0].modbusWrite(0x00);
  //   modulos[0].endTransmission();
  //   modulos[0].beginTransmission(17922,2);//42498//18433//17921
  //   modulos[0].modbusWrite(0x00);
  //   modulos[0].modbusWrite(0x00);
  //   modulos[0].endTransmission();
  //   modulos[0].stop();
  // }
  Serial.println();
}
