#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include <Arduino.h>

enum RobotData {
  PosX = 0,
  PosY = 1,
  State = 2
};

enum VisionData {
  Angle = 0,
  Distance = 1
};

enum VisionType {
  Front = 0,
  Omni = 1
};

class SerialParser {
  private:
    String command;
    int robot1PosX = -1;
    int robot1PosY = -1;
    int robot1State = -1;
    int robot2PosX = -1;
    int robot2PosY = -1;
    int robot2State = -1;
    int visionFrontAngle = -1;
    int visionFrontDistance = -1;
    int visionOmniAngle = -1;
    int visionOmniDistance = -1;
    String mapper[50][2];
    String *m;

    void applyConfig();

  public:
    int mapperLength = 50;
    void parse(String serialData, char separator);
    void init(String *mapperConfig, int size);
    String getValue(String key);
    String* getMap();
};

#endif