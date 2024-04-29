#pragma once

#include <cstdint>

class SensorTask {
  public:
    using DataUpdatedCBR = void(*)(float temp, float humi, float pres, void* pUsrCtx);

  public:
    void Start(DataUpdatedCBR cbr, void* pUsrCtx);
};
