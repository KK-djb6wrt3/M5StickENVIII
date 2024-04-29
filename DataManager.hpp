#pragma once

#include <ctime>

struct SensorData_t {
  float temp;
  float humi;
  float pres;
};

class DataManager {
  public:
    static constexpr uint32_t MAX_ENTRIES = 160U;
  public:
    virtual ~DataManager(void);
    static void Initialize(void);
    static void AddData(float temp, float humi, float pres);
    static int32_t GetDataNum(void);
    static bool GetData(int32_t index, SensorData_t* pData);
    static bool GetLastData(SensorData_t* pData);
    static time_t GetLastAddTime(void);
  private:
    DataManager(void);
};
