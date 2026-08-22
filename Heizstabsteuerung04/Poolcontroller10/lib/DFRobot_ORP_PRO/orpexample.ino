/*!
  * @file gainORP.ino
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V0.1
  * @date        2021-06-17
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_ORP_PRO
  */
#include "DFRobot_ORP_PRO.h"
#define analog_pin A1
#define ADC_BIT 1024.0
#define reference_voltage 5000

float analogval; 

DFRobot_ORP_PRO ORP(/*参考偏置电压*/2470/*mV*/);
void setup() {
  Serial.begin(115200);
  //设置电压参考
  ORP.setCalibration(ORP.calibrate(/*短接 ORP 的电压值*/ 3323 /*mV*/));

  Serial.print("calibration is ：");
  Serial.print(ORP.getCalibration());
  Serial.println("mV");
}

void loop() {
  analogval = analogRead(analog_pin) / ADC_BIT * reference_voltage;
  Serial.print("ORP value is ：");
  Serial.print(ORP.getORP(/*探头电压*/ analogval /*mV*/));
  Serial.println("mV");
  delay(1000);
}
