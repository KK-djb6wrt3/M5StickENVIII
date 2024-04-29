#include <M5StickC.h>
#include <cfloat>
#include <vector>

#include "SensorTask.hpp"
#include "ModeCheckTask.hpp"
#include "DataManager.hpp"

// Types & Constants
static constexpr uint32_t EVF_DATA_UPDATED = (1U << 0);
static constexpr uint32_t INTERVAL_MS = (2U * 60U * 1000U);  //ms;2min
static constexpr uint8_t PIN_LED = 10;
static constexpr int32_t GRAPH_MARGIN_L = 0;
static constexpr int32_t GRAPH_MARGIN_R = 0;
static constexpr int32_t GRAPH_MARGIN_T = 0;
static constexpr int32_t GRAPH_MARGIN_B = 0;
static constexpr int32_t GRAPH_ADJ_UNIT = 5;
static constexpr uint16_t TFT_DARKORANGE = 0x82C0;      /* 128,  90,   0 */
static constexpr uint16_t TFT_DARKGREENYELLOW = 0x5C00; /*  90, 128,   0 */
static constexpr uint32_t INTERVAL_APO_MS = (60U * 1000U);//ms;1min


struct Point_t {
  int16_t x;
  int16_t y;
};
using Points = std::vector<Point_t>;
enum class DataType {
  Temperature,
  Humidity,
  Pressure,
};
using DT = DataType;

// static variables
static SensorTask s_sensor;
static ModeCheckTask s_modeChk;
static QueueHandle_t s_hQueData = nullptr;
static QueueHandle_t s_hQueMode = nullptr;
static Mode s_eMode = Mode::Normal;
static SensorData_t s_data{};
static uint32_t s_tAPO = 0U;


// subroutines
static inline uint32_t sec2ms(time_t sec) {
  return (sec * 1000U);
}

static inline uint64_t ms2us(uint32_t ms) {
  return (ms * 1000U);
}

static inline uint32_t getElapsedTime(void) {
  return sec2ms(std::time(nullptr) - DataManager::GetLastAddTime());
}

static inline void resetAPO(void) {
  s_tAPO = (millis() + INTERVAL_APO_MS);
}

static inline void flashLED(void) {
  digitalWrite(PIN_LED, LOW);
  delay(10);
  digitalWrite(PIN_LED, HIGH);
}

static void drawTemp(float temp, bool isValid = true) {
  M5.Lcd.setTextSize(1);

  M5.Lcd.setTextFont(6);
  M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
  M5.Lcd.setCursor(8, 8);
  if (isValid) {
    const auto n = static_cast<int>(temp);
    M5.Lcd.printf("%2d", n);

    const auto d = static_cast<int>(10.0F * (temp - static_cast<float>(n)));
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(64, 30);
    M5.Lcd.printf(".%d", d);
  } else {
    M5.Lcd.print("--");
  }
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(68, 10);
  M5.Lcd.print(F("C"));
}

static void drawHumi(float humi, bool isValid = true) {
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextFont(6);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setCursor(8, 56);
  if (isValid) {
    const auto n = static_cast<int>(humi);
    M5.Lcd.printf("%2d", n);

    const auto d = static_cast<int>(10.0F * (humi - static_cast<float>(n)));
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(64, 78);
    M5.Lcd.printf(".%d", d);
  } else {
    M5.Lcd.print("--");
  }
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(68, 58);
  M5.Lcd.println(F("%"));
}

static void drawPress(float pres, bool isValid = true) {
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  M5.Lcd.setCursor(8, 96);
  if (isValid) {
    M5.Lcd.printf("%4d", static_cast<int>(std::round(pres)));
  } else {
    M5.Lcd.print("----");
  }
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(56, 124);
  M5.Lcd.println(F("hPa"));
}

static void drawBattery(void) {
  const auto vbat = ((M5.Axp.GetVbatData() * 1.1) / 1000.0);
  auto battGauge = static_cast<int32_t>(std::round((vbat - 3.2) / 0.9 * 100.0));
  Serial.printf("vbat:%.2f battGauge:%d\n", vbat, battGauge);
  if (battGauge > 100) {
    battGauge = 100;
  } else if (battGauge < 0) {
    battGauge = 0;
  }
  const auto isCharging = (M5.Axp.GetBatteryChargingStatus() & (1 << 6));
  const auto isVBUS = (M5.Axp.GetInputPowerStatus() & (1 << 5));
  M5.Lcd.setCursor(4, (M5.Lcd.height() - 16));
  M5.Lcd.setTextSize(1);
  if (isCharging) {
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
  } else if (isVBUS) {
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  } else {
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  }
  M5.Lcd.printf("%3lu%%", battGauge);
}

static inline int32_t getGraphWidth(void) {
  return (M5.Lcd.width() - (GRAPH_MARGIN_L + GRAPH_MARGIN_R));
}
static inline int32_t getGraphHeight(void) {
  return (M5.Lcd.height() - (GRAPH_MARGIN_T + GRAPH_MARGIN_B));
}

static Points prepareGraph(DataType eType, int32_t width, int32_t height, int32_t* pMax, int32_t* pMin) {
  Points ary;

  const auto num = DataManager::GetDataNum();
  if (num > 1) {
    auto fMin = FLT_MAX;
    auto fMax = FLT_MIN;
    const auto getValue = [&eType](int32_t idx) {
      auto val = FLT_MIN;
      SensorData_t data{};
      if (DataManager::GetData(idx, &data)) {
        switch (eType) {
          case DT::Temperature:
            val = data.temp;
            break;
          case DT::Humidity:
            val = data.humi;
            break;
          case DT::Pressure:
            val = data.pres;
            break;
          default:
            //NOP
            break;
        }
        return val;
      }
    };

    for (int32_t i = 0; i < num; i++) {
      const auto value = getValue(i);
      if (value > fMax) {
        fMax = value;
      }
      if (value < fMin) {
        fMin = value;
      }
    }

    auto maxVal = static_cast<int32_t>(std::ceil(fMax));
    auto minVal = static_cast<int32_t>(std::floor(fMin));

    {  // align to adjust unit
      int32_t mod = 0;

      mod = (maxVal % GRAPH_ADJ_UNIT);
      if (mod != 0) {
        maxVal += (GRAPH_ADJ_UNIT - mod);
      }

      mod = (minVal % GRAPH_ADJ_UNIT);
      if (mod != 0) {
        minVal -= mod;
      }

      if (minVal == maxVal) {
        maxVal += GRAPH_ADJ_UNIT;
      }

      if (pMax != nullptr) {
        *pMax = maxVal;
      }
      if (pMin != nullptr) {
        *pMin = minVal;
      }
    }

    const auto diff = (maxVal - minVal);
    const auto calcY = [&](int32_t idx) {
      auto y = (height - (((getValue(idx) - minVal) * height) / diff));
      const auto yMax = (height + GRAPH_MARGIN_T);
      if (y > yMax) {
        y = yMax;
      } else if (y < GRAPH_MARGIN_T) {
        y = GRAPH_MARGIN_T;
      }
      return y;
    };
    const auto calcX = [&](int32_t idx) {
      auto x = (GRAPH_MARGIN_L + ((idx * width) / (num - 1)));
      const auto xMax = (width + GRAPH_MARGIN_L);
      if (x > xMax) {
        x = xMax;
      } else if (x < GRAPH_MARGIN_L) {
        x = GRAPH_MARGIN_L;
      }
      return x;
    };
    for (int32_t i = 0; i < num; i++) {
      Point_t xy{ calcX(i), calcY(i) };
      ary.push_back(xy);
    }
  }
  return ary;
}

static void drawHLine(int32_t width, int32_t height, int32_t maxVal, int32_t minVal, uint16_t color) {
  const auto numHLine = ((maxVal - minVal) / GRAPH_ADJ_UNIT);
  if (numHLine > 1) {
    const auto hLineGap = (height / numHLine);
    for (int32_t i = 1; i < numHLine; i++) {
      M5.Lcd.drawFastHLine(GRAPH_MARGIN_L, (GRAPH_MARGIN_T + (i * hLineGap)), width, color);
    }
  }
}

static void drawMaxMinVal(int32_t x, int32_t maxVal, int32_t minVal, uint16_t color) {
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(color, TFT_NAVY);
  M5.Lcd.setCursor(x, GRAPH_MARGIN_T);
  M5.Lcd.print(maxVal);
  M5.Lcd.setCursor(x, M5.Lcd.height() - (GRAPH_MARGIN_B + 8));
  M5.Lcd.print(minVal);
}

static void drawPressGraph(void) {
  const auto width = getGraphWidth();
  const auto height = getGraphHeight();
  M5.Lcd.fillRect(GRAPH_MARGIN_L, GRAPH_MARGIN_T, width, height, TFT_NAVY);

  int32_t maxPres = 0;
  int32_t minPres = 0;
  const auto&& aryP = prepareGraph(DT::Pressure, width, height, &maxPres, &minPres);

  drawHLine(width, height, maxPres, minPres, TFT_DARKGREENYELLOW);
  drawMaxMinVal(4, maxPres, minPres, TFT_DARKGREENYELLOW);

  const auto num = aryP.size();
  for (size_t i = 1; i < num; i++) {
    const auto& xy0 = aryP[i - 1];
    const auto& xy1 = aryP[i];
    M5.Lcd.drawLine(xy0.x, xy0.y, xy1.x, xy1.y, TFT_GREENYELLOW);
  }
}

static void drawTempHumiGraph(void) {
  const auto width = getGraphWidth();
  const auto height = getGraphHeight();
  M5.Lcd.fillRect(GRAPH_MARGIN_L, GRAPH_MARGIN_T, width, height, TFT_NAVY);

  // temperature
  int32_t maxTemp = 0;
  int32_t minTemp = 0;
  const auto&& aryT = prepareGraph(DT::Temperature, width, height, &maxTemp, &minTemp);
  drawHLine(width, height, maxTemp, minTemp, TFT_DARKORANGE);
  drawMaxMinVal(4, maxTemp, minTemp, TFT_DARKORANGE);

  // humidity
  int32_t maxHumi = 0;
  int32_t minHumi = 0;
  const auto&& aryH = prepareGraph(DT::Humidity, width, height, &maxHumi, &minHumi);
  drawHLine(width, height, maxHumi, minHumi, TFT_DARKCYAN);
  drawMaxMinVal((M5.Lcd.width() - 12), maxHumi, minHumi, TFT_DARKCYAN);

  {
    const auto num = aryT.size();
    for (size_t i = 1; i < num; i++) {
      const auto& xy0 = aryT[i - 1];
      const auto& xy1 = aryT[i];
      M5.Lcd.drawLine(xy0.x, xy0.y, xy1.x, xy1.y, TFT_ORANGE);
    }
  }

  {
    const auto num = aryH.size();
    for (size_t i = 1; i < num; i++) {
      const auto& xy0 = aryH[i - 1];
      const auto& xy1 = aryH[i];
      M5.Lcd.drawLine(xy0.x, xy0.y, xy1.x, xy1.y, TFT_CYAN);
    }
  }
}

static void enterSleep(uint32_t interval_ms) {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
  esp_sleep_enable_timer_wakeup(ms2us(interval_ms));
  esp_deep_sleep_start();
}

static void updateLCD(void) {
  const auto hasData = (DataManager::GetDataNum() > 0);
  switch (s_eMode) {
    case Mode::Normal:
      drawTemp(s_data.temp, hasData);
      drawHumi(s_data.humi, hasData);
      drawPress(s_data.pres, hasData);
      drawBattery();
      break;
    case Mode::PressGraph:
      drawPressGraph();
      break;
    case Mode::TempHumiGraph:
      drawTempHumiGraph();
      break;
    default:
      break;
  }
}


// callbacks
static void onDataUpdatedWake(float temp, float humi, float pres, void* pUsrCtx) {
  auto& hEvent = *reinterpret_cast<EventGroupHandle_t*>(pUsrCtx);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  flashLED();
  DataManager::AddData(temp, humi, pres);
  xEventGroupSetBits(hEvent, EVF_DATA_UPDATED);
}

static void onDataUpdated(float temp, float humi, float pres, void* pUsrCtx) {
  if ((DataManager::GetDataNum() == 0) || (getElapsedTime() >= INTERVAL_MS)) {
    DataManager::AddData(temp, humi, pres);
  }
  SensorData_t data{ temp, humi, pres };
  xQueueSend(s_hQueData, &data, 0);
}

static void onModeChanged(Mode eMode, void* pUsrCtx) {
  xQueueSend(s_hQueMode, &eMode, 0);
}


//arduino
void setup() {
  static const auto tStart = millis();
  setCpuFrequencyMhz(40);

  DataManager::Initialize();

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    // awaken by timer. getting data, then sleep again
    auto hEvData = xEventGroupCreate();
    s_sensor.Start(onDataUpdatedWake, &hEvData);
    xEventGroupWaitBits(hEvData, EVF_DATA_UPDATED, pdTRUE, pdTRUE, portMAX_DELAY);
    vEventGroupDelete(hEvData);

    const auto tSpent = (millis() - tStart);
    enterSleep(INTERVAL_MS - tSpent);
  } else {
    s_hQueData = xQueueCreate(8, sizeof(SensorData_t));
    s_hQueMode = xQueueCreate(8, sizeof(Mode));
    M5.begin();
    M5.Axp.ScreenBreath(8);

    s_sensor.Start(onDataUpdated, nullptr);
    s_modeChk.Start(onModeChanged, nullptr);
    M5.Lcd.setRotation(0);
    SensorData_t data{};
    if (DataManager::GetLastData(&data)) {
      s_data = data;
    }
    updateLCD();
    resetAPO();
  }
}

void loop() {
  auto eMode = Mode::Unknown;
  if (xQueueReceive(s_hQueMode, &eMode, 0) == pdTRUE) {
    if (eMode != s_eMode) {
      constexpr uint8_t INVALID_DIR = 0xFFU;
      auto dir = INVALID_DIR;
      switch (eMode) {
        case Mode::Normal:
          dir = 0;
          break;
        case Mode::PressGraph:
          dir = 1;
          break;
        case Mode::TempHumiGraph:
          dir = 3;
          break;
        default:
          dir = INVALID_DIR;
          break;
      }
      if (dir != INVALID_DIR) {
        s_eMode = eMode;
        M5.Lcd.setRotation(dir);
        M5.Lcd.fillScreen(TFT_BLACK);
        updateLCD();
        resetAPO();
      }
    }
  }

  SensorData_t data{};
  if (xQueueReceive(s_hQueData, &data, 0) == pdTRUE) {
    s_data = data;
    updateLCD();
  }

  const auto isAPO = (millis() >= s_tAPO);
  if (isAPO || M5.BtnB.wasPressed() ) {
    M5.Axp.ScreenSwitch(false);
    M5.Axp.SetLDO2(false);
    M5.Axp.SetLDO3(false);
    enterSleep(INTERVAL_MS - getElapsedTime());
  }

  M5.update();
  delay(1);
}
