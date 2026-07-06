#pragma once

#include <stdint.h>

// XXX: Ideally this would just be stored as a uint16_t value instead
//      of holding onto intermediate stuff. Making each of these just
//      a function would improve performance on embedded.

class LTC681xCommand {
 public:
  virtual uint16_t toValue() const = 0;
};

enum class AdcMode : uint8_t {
  k422 = 0,
  k27k = 1,
  k7k = 2,
  k26 = 3,
  k1k = 0,
  k14k = 1,
  k3k = 2,
  k2k = 3
};
enum class CellSelection : uint8_t {
  kAll = 0,
  k1_7 = 1,
  k2_8 = 2,
  k3_9 = 3,
  k4_10 = 4,
  k5_11 = 5,
  k6_12 = 6
};
enum class PullDirection : uint8_t { kPullDown = 0, kPullUp = 1 };
enum class SelfTestMode : uint8_t { kSelfTest1 = 1, kSelfTest2 = 2 };
enum class GpioSelection : uint8_t {
  kAll = 0,
  k1 = 1,
  k2 = 2,
  k3 = 3,
  k4 = 4,
  k5 = 5,
  kRef = 6
};
enum class StatusGroupSelection : uint8_t {
  kAll = 0,
  kSC = 1,
  kITMP = 2,
  kVA = 3,
  kVD = 4
};

class WriteConfigurationGroupA : public LTC681xCommand {
  uint16_t toValue() const { return 0x0001; }
};
using WRCFGA = WriteConfigurationGroupA;

class WriteConfigurationGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x0024; }
};
using WRCFGB = WriteConfigurationGroupB;

class ReadConfigurationGroupA : public LTC681xCommand {
  uint16_t toValue() const { return 0x0002; }
};
using RDCFGA = ReadConfigurationGroupA;

class ReadConfigurationGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x0025; }
};
using RDCFGB = ReadConfigurationGroupB;

class ReadCellVoltageGroupA : public LTC681xCommand {
  uint16_t toValue() const { return 0x0004; }
};
using RDCVA = ReadCellVoltageGroupA;

class ReadCellVoltageGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x0006; }
};
using RDCVB = ReadCellVoltageGroupB;

class ReadCellVoltageGroupC : public LTC681xCommand {
  uint16_t toValue() const { return 0x0008; }
};
using RDCVC = ReadCellVoltageGroupC;

class ReadCellVoltageGroupD : public LTC681xCommand {
  uint16_t toValue() const { return 0x000A; }
};
using RDCVD = ReadCellVoltageGroupD;

class ReadCellVoltageGroupE : public LTC681xCommand {
  uint16_t toValue() const { return 0x0009; }
};
using RDCVE = ReadCellVoltageGroupE;

class ReadCellVoltageGroupF : public LTC681xCommand {
  uint16_t toValue() const { return 0x000B; }
};
using RDCVF = ReadCellVoltageGroupF;

class ReadAuxiliaryGroupA : public LTC681xCommand {
  uint16_t toValue() const { return 0x000C; }
};
using RDAUXA = ReadAuxiliaryGroupA;

class ReadAuxiliaryGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x000E; }
};
using RDAUXB = ReadAuxiliaryGroupB;

class ReadAuxiliaryGroupC : public LTC681xCommand {
  uint16_t toValue() const { return 0x000D; }
};
using RDAUXC = ReadAuxiliaryGroupC;

class ReadAuxiliaryGroupD : public LTC681xCommand {
  uint16_t toValue() const { return 0x000F; }
};
using RDAUXD = ReadAuxiliaryGroupD;

class ReadStatusGroupA : public LTC681xCommand {
  uint16_t toValue() const { return 0x0010; }
};
using RDSTATA = ReadStatusGroupA;

class ReadStatusGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x0012; }
};
using RDSTATB = ReadStatusGroupB;

class WriteSControlGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0016; }
};
using WRSCTRL = WriteSControlGroup;

class WritePWMGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0020; }
};
using WRPWM = WritePWMGroup;

class WritePWMGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x001C; }
};
using WRPSB = WritePWMGroupB;

class ReadSControlGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0016; }
};
using RDSCTRL = ReadSControlGroup;

class ReadPWMGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0022; }
};
using RDPWM = ReadPWMGroup;

class ReadPWMGroupB : public LTC681xCommand {
  uint16_t toValue() const { return 0x001E; }
};
using RDPSB = ReadPWMGroupB;

class StartSControlPulsing : public LTC681xCommand {
  uint16_t toValue() const { return 0x0019; }
};
using STSCTRL = StartSControlPulsing;

class ClearSControlGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0018; }
};
using CLRSCTRL = ClearSControlGroup;

class StartCellVoltageADC : public LTC681xCommand {
 public:
  StartCellVoltageADC(AdcMode a, bool d, CellSelection c)
      : adcMode(a), dischargePermitted(d), cellSelection(c) {}
  AdcMode adcMode;
  bool dischargePermitted;
  CellSelection cellSelection;
  uint16_t toValue() const {
    return 0x0260 | (((uint16_t)adcMode) << 7) |
           (dischargePermitted ? 0x0010 : 0x0000) | ((uint16_t)cellSelection);
  }
};
using ADCV = StartCellVoltageADC;

class StartOpenWireADC : public LTC681xCommand {
 public:
  StartOpenWireADC(AdcMode a, PullDirection p, bool d, CellSelection c)
    : adcMode(a), pull(p), dischargePermitted(d), cellSelection(c) {}
  AdcMode adcMode;
  PullDirection pull;
  bool dischargePermitted;
  CellSelection cellSelection;
  uint16_t toValue() const {
    return 0x0228 | ((uint16_t)adcMode << 7) | ((uint16_t)pull << 6) |
           (dischargePermitted ? 0x0010 : 0x0000) | ((uint16_t)cellSelection);
  }
};
using ADOW = StartOpenWireADC;

class StartSelfTestCellVoltage : public LTC681xCommand {
 public:
  StartSelfTestCellVoltage(AdcMode a, SelfTestMode t) : adcMode(a), testMode(t) {}
  AdcMode adcMode;
  SelfTestMode testMode;
  uint16_t toValue() const {
    return 0x0207 | ((uint16_t)adcMode << 7) | ((uint16_t)testMode << 5);
  }
};
using CVST = StartSelfTestCellVoltage;

class StartOverlapCellVoltage : public LTC681xCommand {
 public:
  StartOverlapCellVoltage(AdcMode a, bool d) : adcMode(a), dischargePermitted(d) {}
  AdcMode adcMode;
  bool dischargePermitted;
  uint16_t toValue() const {
    return 0x0201 | ((uint16_t)adcMode << 7) | ((uint16_t)dischargePermitted << 4);
  }
};
using ADOL = StartOverlapCellVoltage;

class StartGpioADC : public LTC681xCommand {
 public:
  StartGpioADC(AdcMode a, GpioSelection g) : adcMode(a), gpioSelection(g) {}
  AdcMode adcMode;
  GpioSelection gpioSelection;
  uint16_t toValue() const {
    return 0x0460 | ((uint16_t)adcMode << 7) | ((uint16_t)gpioSelection);
  }
};
using ADAX = StartGpioADC;

class StartGpioADCWithRedundancy : public LTC681xCommand {
 public:
  StartGpioADCWithRedundancy(AdcMode a, GpioSelection g) : adcMode(a), gpioSelection(g) {}
  AdcMode adcMode;
  GpioSelection gpioSelection;
  uint16_t toValue() const {
    return 0x0400 | ((uint16_t)adcMode << 7) | ((uint16_t)gpioSelection);
  }
};
using ADAXD = StartGpioADCWithRedundancy;

class StartSelfTestGpio : public LTC681xCommand {
 public:
  StartSelfTestGpio(AdcMode a, SelfTestMode t) : adcMode(a), testMode(t) {}
  AdcMode adcMode;
  SelfTestMode testMode;
  uint16_t toValue() const {
    return 0x0400 | ((uint16_t)adcMode << 7) | ((uint16_t)testMode);
  }
};
using AXST = StartSelfTestGpio;

class StartStatusGroupConversion : public LTC681xCommand {
 public:
  StartStatusGroupConversion(AdcMode a, StatusGroupSelection s) : adcMode(a), statusSelection(s) {}
  AdcMode adcMode;
  StatusGroupSelection statusSelection;
  uint16_t toValue() const {
    return 0x0468 | ((uint16_t)adcMode << 7) | ((uint16_t)statusSelection);
  }
};
using ADSTAT = StartStatusGroupConversion;

class StartStatusGroupConversionWithRedundancy : public LTC681xCommand {
 public:
  StartStatusGroupConversionWithRedundancy(AdcMode a, StatusGroupSelection s) : adcMode(a), statusSelection(s) {}
  AdcMode adcMode;
  StatusGroupSelection statusSelection;
  uint16_t toValue() const {
    return 0x0408 | ((uint16_t)adcMode << 7) | ((uint16_t)statusSelection);
  }
};
using ADSTATD = StartStatusGroupConversionWithRedundancy;

class StartSelfTestStatusGroup : public LTC681xCommand {
 public:
  StartSelfTestStatusGroup(AdcMode a, SelfTestMode t) : adcMode(a), testMode(t) {}
  AdcMode adcMode;
  SelfTestMode testMode;
  uint16_t toValue() const {
    return 0x040F | ((uint16_t)adcMode << 7) | ((uint16_t)testMode);
  }
};
using STATST = StartSelfTestStatusGroup;

class StartCombinedCellVoltageGpioConversion : public LTC681xCommand {
 public:
  StartCombinedCellVoltageGpioConversion(AdcMode a, bool d) : adcMode(a), dischargePermitted(d) {}
  AdcMode adcMode;
  bool dischargePermitted;
  uint16_t toValue() const {
    return 0x086F | ((uint16_t)adcMode << 7) | ((uint16_t)dischargePermitted << 4);
  }
};
using ADCVAX = StartCombinedCellVoltageGpioConversion;

class StartCombinedCellVoltageSCConversion : public LTC681xCommand {
 public:
  StartCombinedCellVoltageSCConversion(AdcMode a, bool d) : adcMode(a), dischargePermitted(d) {}
  AdcMode adcMode;
  bool dischargePermitted;
  uint16_t toValue() const {
    return 0x086F | ((uint16_t)adcMode << 7) | ((uint16_t)dischargePermitted << 4);
  }
};
using ADCVSC = StartCombinedCellVoltageSCConversion;

class ClearVoltageGroups : public LTC681xCommand {
  uint16_t toValue() const { return 0x0711; }
};
using CLRCELL = ClearVoltageGroups;

class ClearAuxiliaryGroups : public LTC681xCommand {
  uint16_t toValue() const { return 0x0712; }
};
using CLRAUX = ClearAuxiliaryGroups;

class ClearStatusGroups : public LTC681xCommand {
  uint16_t toValue() const { return 0x0713; }
};
using CLRSTAT = ClearStatusGroups;

class PollADCStatus : public LTC681xCommand {
  uint16_t toValue() const { return 0x0714; }
};
using PLADC = PollADCStatus;

class DiagnoseMux : public LTC681xCommand {
  uint16_t toValue() const { return 0x0715; }
};
using DIAGN = DiagnoseMux;

class WriteCommGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0721; }
};
using WRCOMM = WriteCommGroup;

class ReadCommGroup : public LTC681xCommand {
  uint16_t toValue() const { return 0x0722; }
};
using RDCOMM = ReadCommGroup;

class StartComm : public LTC681xCommand {
  uint16_t toValue() const { return 0x0723; }
};
using STCOMM = StartComm;
