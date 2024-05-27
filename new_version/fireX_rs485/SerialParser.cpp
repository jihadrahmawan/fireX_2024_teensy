#include "SerialParser.h"

void SerialParser::parse(String serialData, char separator) {
  for(int dataIdx = 0; dataIdx < this->mapperLength; dataIdx++) {
    int strIdx[] = { 0,-1};
    int maxIdx = serialData.length() - 1;
    int found = 0;

    for(int idx = 0; idx <= maxIdx && found <= dataIdx; idx++) {
      if(serialData.charAt(idx) == separator || idx == maxIdx) {
        found++;
        strIdx[0] = strIdx[1] + 1;
        strIdx[1] = (idx == maxIdx) ? idx + 1: idx;
      }
    }

    this->mapper[dataIdx][1] = found > dataIdx ? serialData.substring(strIdx[0], strIdx[1]) : "";
  }
}

void SerialParser::init(String *mapperConfig, int size) {
  this->m = mapperConfig;
  this->mapperLength = size;
  SerialParser::applyConfig();
}

String SerialParser::getValue(String key) {
  for(int idx = 0; idx < this->mapperLength; idx++) {
    if(this->mapper[idx][0] == key) {
      return this->mapper[idx][1];
    }
  }
}

String* SerialParser::getMap() {
  return this->m;
}

void SerialParser::applyConfig(){
  for(int idx = 0; idx < this->mapperLength; idx++) {
    this->mapper[idx][0] = this->m[idx];
    this->mapper[idx][1] = "";
  }
}