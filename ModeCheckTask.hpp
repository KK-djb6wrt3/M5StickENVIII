#pragma once

#include <cstdint>

enum class Mode {
  Unknown,
  Normal,
  PressGraph,
  TempHumiGraph,
};

class ModeCheckTask {
  public:
    using ChangeModeCBR = void(*)(Mode eMode, void* pUsrCtx);
    void Start(ChangeModeCBR cbr, void* pUsrCtx);
};
