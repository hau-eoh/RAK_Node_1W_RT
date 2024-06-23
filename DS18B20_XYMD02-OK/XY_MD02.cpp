#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
#include "XY_MD02.h"
ModbusMaster node;
uint8_t result;
unsigned long lastMillis = 0;
void init_xy()
{
  Serial.begin(115200);
  Serial.print("\r\n*****************RAK3172_CANOPUS*******************");
  delay(1000);
  Serial_Canopus.begin(9600, SERIAL_8N1);
  init_io();
  enable_Vrs485();
  node.begin(1, Serial_Canopus);
}


uint16_t read_temp(){
    long currentMillis = millis();
  if (currentMillis - lastMillis > 1000) {
    uint8_t result = node.readInputRegisters(0x01, 2);
    // if (getResultMsg(&node, result)) {
    //   float res_dbl = node.getResponseBuffer(0) / 10.0;
    //   String res = "Temperature: " + String(res_dbl) + " C\r\n";
    // //   res_dbl = node.getResponseBuffer(1) / 10.0;
    // //   Serial.println(node.getResponseBuffer(1),HEX);
    // //   res += "Humidity: " + String(res_dbl) + " %";
    // //   Serial.println(res);
    // }
    lastMillis = currentMillis;
  }
  return node.getResponseBuffer(0);
}
uint16_t read_humi(){
    long currentMillis = millis();
  if (currentMillis - lastMillis > 1000) {
    uint8_t result = node.readInputRegisters(0x01, 2);
    // if (getResultMsg(&node, result)) {
    // //   Serial.println();
    // //   float res_dbl = node.getResponseBuffer(0) / 10.0;
    // //   Serial.println(node.getResponseBuffer(0),HEX);
    // //   String res = "Temperature: " + String(res_dbl) + " C\r\n";
    //   float res_dbl = node.getResponseBuffer(1) / 10.0;
    // }
    lastMillis = currentMillis;
  }
  return node.getResponseBuffer(1);
}
// void loop()
// {
//   //***************READ node 1**************************
//    // slave ID node
//   long currentMillis = millis();
//   if (currentMillis - lastMillis > 1000) {
//     uint8_t result = node.readInputRegisters(0x01, 2);
//     if (getResultMsg(&node, result)) {
//       Serial.println();
//       float res_dbl = node.getResponseBuffer(0) / 10.0;
//       Serial.println(node.getResponseBuffer(0),HEX);
//       String res = "Temperature: " + String(res_dbl) + " C\r\n";
//       res_dbl = node.getResponseBuffer(1) / 10.0;
//       Serial.println(node.getResponseBuffer(1),HEX);
//       res += "Humidity: " + String(res_dbl) + " %";
//       Serial.println(res);
//     }
//     lastMillis = currentMillis;
//   }
// }

bool getResultMsg(ModbusMaster *node, uint8_t result) {
  String tmpstr2 = "\r\n";
  switch (result) {
    case node->ku8MBSuccess:
      return true;
      break;
    case node->ku8MBIllegalFunction:
      tmpstr2 += "Illegal Function";
      break;
    case node->ku8MBIllegalDataAddress:
      tmpstr2 += "Illegal Data Address";
      break;
    case node->ku8MBIllegalDataValue:
      tmpstr2 += "Illegal Data Value";
      break;
    case node->ku8MBSlaveDeviceFailure:
      tmpstr2 += "Slave Device Failure";
      break;
    case node->ku8MBInvalidSlaveID:
      tmpstr2 += "Invalid Slave ID";
      break;
    case node->ku8MBInvalidFunction:
      tmpstr2 += "Invalid Function";
      break;
    case node->ku8MBResponseTimedOut:
      tmpstr2 += "Response Timed Out";
      break;
    case node->ku8MBInvalidCRC:
      tmpstr2 += "Invalid CRC";
      break;
    default:
      tmpstr2 += "Unknown error: " + String(result);
      break;
  }
  Serial.println(tmpstr2);
  return false;
}
