#include "ModeCheckTask.hpp"
#include <M5StickC.h>

static ModeCheckTask::ChangeModeCBR s_cbr = nullptr;
static void* s_pUsrCtx = nullptr;

static Mode checkPitch(float pitch) {
  auto eMode = Mode::Normal;
  if (pitch > 60.0F) {
    eMode = Mode::TempHumiGraph;
  }
  else if (pitch < -60.0F) {
    eMode = Mode::PressGraph;
  }
  return eMode;
}

static void checkModeTask(void* pArg) {
  float pitch = 0.0F;
  float roll = 0.0F;
  float yaw = 0.0F;
  int32_t count = 0;
  auto ePrevMode = Mode::Unknown;
  auto eCurrMode = Mode::Normal;
  M5.Imu.Init();
  for (;;) {
    M5.Imu.getAhrsData(&pitch, &roll, &yaw);
    const auto eTmpMode = checkPitch(pitch);
    if (eTmpMode == ePrevMode) {
      if (++count >= 300) {
        count = 0;
        if (eCurrMode != eTmpMode) {
          eCurrMode = eTmpMode;
          if (s_cbr != nullptr) {
            (*s_cbr)(eCurrMode, s_pUsrCtx);
          }
        }
      }
    }
    else {
      count = 0;
    }
    ePrevMode = eTmpMode;
    delay(1);

  }
}

void ModeCheckTask::Start(ChangeModeCBR cbr, void* pUsrCtx) {
  s_cbr = cbr;
  s_pUsrCtx = pUsrCtx;

  static bool sIs1st = true;
  if (sIs1st) {
    sIs1st = false;
    xTaskCreate(checkModeTask, "checkModeTask", 4096, nullptr, 2, nullptr);
  }
}
