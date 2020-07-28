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
  Serial.println(F("\t{updateDate} -> Actualiza la fecha en los DSE y en el plc"));
  Serial.println(F("\t{debug} -> Activa/desactiva la informacion de debugeo"));
  Serial.println(F("\t{connectModules} -> Forza la conexion a los DSE"));
  Serial.println(F("\t{printMemory} -> Muestra la cantidad de bytes disponibles en la memoria"));
  Serial.println(F("##########"));
}

void okCallback(){
  Serial.println(F("> OK"));
}
