#include "SensorTask.hpp"
#include <M5UnitENV.h>

static SensorTask::DataUpdatedCBR s_cbr = nullptr;
static void* s_pUsrCtx = nullptr;


static void sensorTask(void* pArg) {
  static SHT3X sSHT3X;
  static QMP6988 sQMP6988;

  static constexpr uint8_t SDA = 0;
  static constexpr uint8_t SCL = 26;

  if (!sQMP6988.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, SDA, SCL, 400000U)) {
    Serial.println("Couldn't find QMP6988");
    while (1) {
      delay(1);
    }
  }

  if (!sSHT3X.begin(&Wire, SHT3X_I2C_ADDR, SDA, SCL, 400000U)) {
    Serial.println("Couldn't find SHT3X");
    while (1) {
      delay(1);
    }
  }

  constexpr uint32_t SETTLING = (8U * 1000U);  //8sec
  for (;;) {
    delay(SETTLING);
    if (sSHT3X.update() && sQMP6988.update()) {
      if (s_cbr != nullptr) {
        (*s_cbr)(sSHT3X.cTemp, sSHT3X.humidity, (static_cast<float>(sQMP6988.pressure) / 100.0F), s_pUsrCtx);
      }
    }
  }
}


void SensorTask::Start(DataUpdatedCBR cbr, void* pUsrCtx) {
  s_cbr = cbr;
  s_pUsrCtx = pUsrCtx;

  auto sIs1st = true;
  if (sIs1st) {
    sIs1st = false;
    xTaskCreate(sensorTask, "sensorTask", 4096, nullptr, 1, nullptr);
  }
}
