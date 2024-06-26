#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
#include "XY_MD02.h"
#include "ds18b20.h"

double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;


uint8_t MAC[6] = { 0x01, 0x02, 0x03, 0x04, 0x06, 0x06 };
uint8_t frame[16];
bool rx_done = false;
bool join_success = false;
bool gateway_ok = false;
uint8_t len;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  Serial_Canopus.begin(9600, SERIAL_8N1);
  Serial.println("RAK3172 TEST");
  Serial.println("------------------------------------------------------");
  /////////////////////// Delay For Detect baudrate- DO NOT DELETE ////////
  delay(1000);
  ////////////////////////////////////////////////////////////////////////
  init_io();        // kích hoạt chân gpio,led
  enable_Vss5();    // Init nguồn 5v
  enable_Vrs485();  // Init cổng rs485
  init_xymd02();    // init XY_MD02
  startTime = millis();
  if (api.lora.nwm.get() != 0)  // Set P2P mode
  {
    Serial.printf("Set Node device work mode %s\r\n", api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }
  Serial.println("P2P Start");
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());                                                   // ID chip
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());                                                     //ID model
  Serial.printf("RUI API Version: %s\r\n", api.system.apiVersion.get().c_str());                                           //Ver API
  Serial.printf("Firmware Version: %s\r\n", api.system.firmwareVersion.get().c_str());                                     //Ver Firmware
  Serial.printf("AT Command Version: %s\r\n", api.system.cliVersion.get().c_str());                                        //Ver AT command
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6), api.lora.pfreq.set(myFreq) ? "Success" : "Fail");  //set tần số
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf, api.lora.psf.set(sf) ? "Success" : "Fail");                //set spreading factor
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw, api.lora.pbw.set(bw) ? "Success" : "Fail");                       //set băng thông
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5), api.lora.pcr.set(cr) ? "Success" : "Fail");               //set code  rate
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble, api.lora.ppl.set(preamble) ? "Success" : "Fail");     //set preamble length
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower, api.lora.ptp.set(txPower) ? "Success" : "Fail");              //set TX power
  Serial.printf("Set the low power mode %s\n\r", api.system.lpm.set(1) ? "Success" : "Fail");
  api.lora.registerPRecvCallback(recv_cb);  // goij hàm recv_cb mỗi khi nhận dữ liệu xong
  api.lora.registerPSendCallback(send_cb);  // gọi hàm send_cb mỗi khi gửi xong
  while (!join_success) {
    uint8_t join_data[1] = { 0x01 };       // node dùng cảm biến rời + onewire
    send_frame(0x01, 0x01, join_data, 1);  // gửi frame request join lên gateway
    Serial.println("Join request Sent");
    //Chờ RX nhận dữ liệu
    if (join_success) {  // nếu nhận đc phản hồi từ gateway thì thoát vòng while và bắt đầu gửi data
      Serial.println("-----------------------               --------------------");
      Serial.println("----------------------- Join Success --------------------");
      Serial.println("-----------------------               --------------------");
    } else {
      Serial.println("Not Responding From GateWay.....");
      Serial.println("Join Fail");
      Serial.println("Retrying.........");
    }
    delay(5000);
  }
}

void loop() {
  float temp = read_temperature();           // biến nhiệt độ từ 1W 16bit
  uint16_t temp_scaled = temp * 10;          //123
  uint8_t temp_data[2];                      // mảng tách giá trị của nhiệt độ từ 1W
  Serial.print("scaled temp: ");
  Serial.println(temp_scaled);
  Serial.print("temp: ");
  Serial.println(temp);
  
  temp_data[0] = temp_scaled & 0xFF;         // LSB//8bit cao
  temp_data[1] = (temp_scaled >> 8) & 0xFF;  // MSB//8 bit thấp
  uint16_t xy_md02_temp = read_temp();       // biến nhiệt độ từ rs485 16bit
  uint8_t tempmd02_data[2];                  // mảng tách giá trị nhiệt độ từ rs485
  tempmd02_data[0] = xy_md02_temp;
  tempmd02_data[1] = (xy_md02_temp >> 8);
  uint16_t xy_md02_humi = read_humi();  // biến độ ẩm từ rs485
  uint8_t humi_data[2];                 // mảng tách giá trị độ ẩm từ rs485
  humi_data[0] = xy_md02_humi;
  humi_data[1] = xy_md02_humi >> 8;
  /*//////////////////////////////////////////

                     Send Frame

  ///////////////////////////////////////////*/
  // int rxDelay = 5000;// Thời gian nghe của RX
  // if(millis() - startTime >= 10*1000)// 10s sẽ chuyển sang RX
  // {
  //   Serial.printf("P2P Rx start for %d millisSeconds\r\n", rxDelay);
  //   startTime = millis();
  //   Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(rxDelay) ? "Success" : "Fail");
  //   delay(rxDelay);
  // }
  delay(30000);
  send_frame(0x03, 0x02, humi_data, 2);      // gửi frame độ ẩm lên gateway
  delay(2000);                               // Chờ cho gateway nhận dữ liệu
  send_frame(0x03, 0x03, tempmd02_data, 2);  // gửi frame nhiệt độ từ rs485
  delay(2000);                               // Chờ cho gateway nhận dữ liệu
  send_frame(0x03, 0x04, temp_data, 2);      // gửi frame nhiệt độ từ 1W
  // delay(30000);
}
/*////////////////////////////////////////////////

                    Funtion                    

///////////////////////////////////////////////////*/
void hexDump(uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i += 16) {  //Duyet qua buffer theo tung nhom 2byte
    char s[len];                            //Mang s: Luu tru ky tu ASCII
    uint8_t iy = 0;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        if (c > 31 && c < 128) {  // mã ascii từ SPACE tới ~
          s[iy++] = c;
        } else {
          s[iy++] = '.';
        }
      }
    }
    String msg = String(s);
    Serial.println(msg);
  }
  Serial.println("Buffer!");
}
void recv_cb(rui_lora_p2p_recv_t data) {
  rx_done = true;
  if (data.BufferSize == 0) {
    Serial.println("Empty buffer.");
    return;
  }
  digitalWrite(LED_RECV, !digitalRead(LED_RECV));
  char buff[92];
  uint8_t respone = buff[5];
  if (respone == 0x00 || 0x01) {
    join_success = true;
  } else {
    join_success = false;
  }
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d", data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);
  hexDump(data.Buffer, data.BufferSize);
  digitalWrite(LED_RECV, !digitalRead(LED_RECV));
}
void send_cb(void) {
  Serial.printf("P2P Set RX Mode: %s\r\n", api.lora.precv(65534) ? "Success" : "Fail");  // bật mode RX, TX vẫn có thể hoạt động được
}
uint8_t crc8(const uint8_t *data, int len) {
  unsigned crc = 0;
  int i, j;
  for (j = len; j; j--, data++) {
    crc ^= (*data << 8);
    for (i = 8; i; i--) {
      if (crc & 0x8000) {
        crc ^= (0x1070 << 3);
      }
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}
void send_frame(uint8_t frame_type, uint8_t function, uint8_t *data, uint8_t data_len) {
  bool send_result = false;
  len = 0;                              //must have for reset len each call function
  frame[len++] = 0xFE;                  // BEGIN
  frame[len++] = 0;                     // LEN (placeholder)
  frame[len++] = frame_type;            // FRAME TYPE
  frame[len++] = function;              // FUNCTION
  memcpy(&frame[len], MAC, 6);          // MAC
  len += 6;                             //Mac address 6 bytes : cộng dồn
  memcpy(&frame[len], data, data_len);  // DATA
  len += data_len;

  //Frame se la tu bat dau tinh tu vi tri sau begin
  frame[1] = len - 2;  // LEN (actual length without BEGIN and END)

  uint8_t crc = crc8(&frame[1], len - 1);
  frame[len++] = crc;   // CRC
  frame[len++] = 0xEF;  // END
  // Truyền frame qua LoRa
  uint8_t retry_count = 0;        // Đếm số lần thử gửi lại.
  const uint8_t max_retries = 5;  // Giới hạn số lần thử gửi lại.
  while (!send_result && retry_count < max_retries) {
    send_result = api.lora.psend(len, frame);                                              // gửi và kiểm tra kết quả gửi
    Serial.printf("P2P send %s\r\n", send_result ? "Success" : "Fail");                    // In kết quả gửi và chạy hàm send_cb
    if (!send_result) {                                                                    // Nếu gửi kh thành công
      Serial.printf("P2P finish Rx mode %s\r\n", api.lora.precv(0) ? "Success" : "Fail");  // tắt mode RX( bật khi chạy hàm send_cb)- bật TX mode
    }
    //Check số lần gửi
    retry_count++;  // Tăng số lần thử gửi lại.
    Serial.print("Check resend: ");
    Serial.println(retry_count);
  }
  // Truyền frame
  Serial.print("Sending frame: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  // Serial.println("Waiting for gateway respone............");
}
void sleep_mode(int time) {
  Serial.print("The timestamp before sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("(Wait 10 seconds or Press any key to wakeup)");
  api.system.sleep.all(time);
  Serial.print("The timestamp after sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
}