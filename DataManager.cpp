#include "DataManager.hpp"
#include <Arduino.h>


static xSemaphoreHandle s_hMutex = nullptr;
static RTC_DATA_ATTR int32_t s_numEntries = 0;
static RTC_DATA_ATTR int32_t s_writeHead = 0;
static RTC_DATA_ATTR int16_t s_pTemperatures[DataManager::MAX_ENTRIES]{};
static RTC_DATA_ATTR int16_t s_pHumidities[DataManager::MAX_ENTRIES]{};
static RTC_DATA_ATTR int16_t s_pPressures[DataManager::MAX_ENTRIES]{};
static RTC_DATA_ATTR time_t s_tPrevAdd = 0U;

static inline int16_t toS16(float val) {
  return static_cast<int16_t>(std::round(val * 10.0F));
}

static inline float toFloat(int16_t val) {
  return (static_cast<float>(val) / 10.0F);
}

static int32_t getActualIndex(int32_t idx) {
  int32_t actIdx = 0;
  if (s_numEntries == DataManager::MAX_ENTRIES) {
    const auto diff = (DataManager::MAX_ENTRIES - s_writeHead);
    if (idx < diff) {
      actIdx = (idx + s_writeHead);
    } else {
      actIdx = (idx - diff);
    }
  } else {
    actIdx = idx;
  }
  return actIdx;
}

void DataManager::Initialize(void) {
  if (s_hMutex == nullptr) {
    s_hMutex = xSemaphoreCreateMutex();
  }
}
void DataManager::AddData(float temp, float humi, float pres) {
  xSemaphoreTake(s_hMutex, portMAX_DELAY);
  auto idx = s_writeHead;
  s_pTemperatures[s_writeHead] = toS16(temp);
  s_pHumidities[s_writeHead] = toS16(humi);
  s_pPressures[s_writeHead] = toS16(pres);
  s_writeHead++;
  if (s_writeHead >= MAX_ENTRIES) {
    s_writeHead = 0;
  }
  if (s_numEntries < MAX_ENTRIES) {
    s_numEntries++;
  }
  s_tPrevAdd = std::time(nullptr);
  xSemaphoreGive(s_hMutex);
}

int32_t DataManager::GetDataNum(void) {
  xSemaphoreTake(s_hMutex, portMAX_DELAY);
  const auto num = s_numEntries;
  xSemaphoreGive(s_hMutex);
  return num;
}

bool DataManager::GetData(int32_t index, SensorData_t* pData) {
  xSemaphoreTake(s_hMutex, portMAX_DELAY);
  const auto isOK = (index < s_numEntries);
  if (isOK) {
    const auto actIdx = getActualIndex(index);
    SensorData_t data{};
    data.temp = toFloat(s_pTemperatures[actIdx]);
    data.humi = toFloat(s_pHumidities[actIdx]);
    data.pres = toFloat(s_pPressures[actIdx]);
    if (pData != nullptr) {
      *pData = data;
    }
  } else {
    Serial.println("GetData failure\n");
  }
  xSemaphoreGive(s_hMutex);
  return isOK;
}

bool DataManager::GetLastData(SensorData_t* pData) {
  xSemaphoreTake(s_hMutex, portMAX_DELAY);
  auto isGot = (s_numEntries > 0);
  if (isGot) {
    if (pData != nullptr) {
      const auto idx = getActualIndex(s_numEntries - 1);
      *pData = SensorData_t{
        toFloat(s_pTemperatures[idx]),
        toFloat(s_pHumidities[idx]),
        toFloat(s_pPressures[idx]),
      };
    }
  }
  xSemaphoreGive(s_hMutex);
  return isGot;
}

time_t DataManager::GetLastAddTime(void) {
  xSemaphoreTake(s_hMutex, portMAX_DELAY);
  const auto t = s_tPrevAdd;
  xSemaphoreGive(s_hMutex);
  return t;
}
