/*
Khi bật nguồn led đỏ nháy 20 lần sau đó dừng lại và sáng led đỏ---Đang chờ join vào gateway
Khi gửi led xanh sẽ sáng( TX Mode)
Khi nhận led vàng sẽ sáng (RX Mode)
3 LED tắt--- Sleep 

Sau mỗi lần gửi data xong, RX ở nghe trong khoảng 20s. Nếu kh có dữ liệu gửi về  sẽ vào chế độ ngủ.
*/



#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
#include "XY_MD02.h"
#include "ds18b20.h"
#define button_pin PA11
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;


uint8_t MAC[6] = { 0x01, 0x02, 0x03, 0x04, 0x06, 0x06 };
uint8_t frame[16];
uint8_t count=0;
bool rx_done = false;
bool join_success = false;
bool gateway_ok = false;
uint8_t len;
uint8_t status_DI = 0x00;
float battery;
uint8_t batt_int;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  // udrv_sys_clock_init();
  // udrv_register_wakeup_callback(&WakeUp);
  // udrv_sys_clock_on();
  Serial_Canopus.begin(115200, SERIAL_8N1);
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
  pinMode(button_pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), ButtonHandle, CHANGE);
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
  Serial.printf("Set the low power mode %s\n\r", api.system.lpm.set(1) ? "Success" : "Fail");                              //set Low Power mode
  api.lora.registerPRecvCallback(recv_cb);  // gọi hàm recv_cb mỗi khi vào mode RX
  api.lora.registerPSendCallback(send_cb);// gọi hàm send_cb mỗi khi gửi xong
  for(int i=0;i<20;i++){
    digitalWrite(LED_SYNC,!digitalRead(LED_SYNC));
    delay(50);
  }
  digitalWrite(LED_SYNC,1);
  while (!join_success) {
    uint8_t join_data[1] = { 0x01 };       // node dùng cảm biến rời + onewire
    send_frame(0x01, 0x01, join_data, 1);  // gửi frame request join lên gateway
    Serial.println("Join request Sent");
    // api.lora.precv(5000);
    delay(6000);
    if(join_success){
      Serial.println("-----------------------               --------------------");
      Serial.println("----------------------- Join Success --------------------");
      Serial.println("-----------------------               --------------------");
      digitalWrite(LED_SYNC,0);
      battery = (api.system.bat.get()/4.2)*100;
      batt_int = battery;
      send_frame(0x03, 0x07, &batt_int, 1);
      delay(2000);
      sensor_frame();//gửi data sensor lần đầu
    }else{
      Serial.println("-----------------------               --------------------");
      Serial.println("------------------- !!! JOIN FAIL !!!! -------------------");
      Serial.println("-----------------------               --------------------");
    }
  }
  if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)WakeUp, RAK_TIMER_PERIODIC) != true) {
    Serial.printf("Creating timer failed.\r\n");
  } 
  else if (api.system.timer.start(RAK_TIMER_0, 60000, (void*)0) != true) {
    Serial.printf("Starting timer failed.\r\n");
  }
}

void loop() {
  sleep_mode();
}
/*////////////////////////////////////////////////

                    Funtion                    

///////////////////////////////////////////////////*/
void WakeUp()
{   
    Serial.println("Hàm Callback ");
    count++;
    if(count==1440){// 24h sẽ gửi pin
      battery = (api.system.bat.get()/4.2)*100;
      batt_int = battery;
      Serial.printf("Battery: %d%% ",batt_int);
      send_frame(0x03, 0x07, &batt_int, 1);
      delay(2000);
      count=0;
    }
    sensor_frame();
}
void ButtonHandle(){
  long pre=0;
  if(millis()-pre>=500){
      pre=millis();
      if(digitalRead(button_pin)==LOW){
        Serial.println("_______________________________  External Interrupt!!!_________________________________________");
        // send_frame(0x03, 0x06, &status_led, 1);
        uint8_t adam_data[4];
        adam_data[0] = 0;
        adam_data[1] = 1;
        adam_data[2] = 0;
        adam_data[3] = 0;
        send_frame(0x03, 0x05, adam_data, 4);
        delay(10000);
        adam_data[1] = 0;
        send_frame(0x03, 0x05, adam_data, 4);
      }else{
        return;
      }
  }
}
void sensor_frame(){// 1W-RT
  float temp = read_temperature();           // biến nhiệt độ từ 1W 16bit
  uint16_t temp_scaled = temp * 10;          //123
  uint8_t temp_data[2];                   // mảng tách giá trị của nhiệt độ từ 1W
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
  send_frame(0x03, 0x02, humi_data, 2);      // gửi frame độ ẩm lên gateway
  delay(2000);                             
  send_frame(0x03, 0x03, tempmd02_data, 2);  // gửi frame nhiệt độ từ rs485
  delay(2000);                             
  send_frame(0x03, 0x04, temp_data, 2);      // gửi frame nhiệt độ từ 1W
}
void hexDump(uint8_t *buf, uint16_t len)
{
    char alphabet[17] = "0123456789ABCDEF";
    for (uint16_t i = 0; i < len; i += 16)
    {
        if (i % 128 == 0)
            Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
        char s[] = "|                                                | |                |\r\n";
        uint8_t ix = 1, iy = 52;
        for (uint8_t j = 0; j < 16; j++)
        {
              if (i + j < len)
              {
                  uint8_t c = buf[i + j];
                  s[ix++] = alphabet[(c >> 4) & 0x0F];
                  s[ix++] = alphabet[c & 0x0F];
                  ix++;
          }
          uint8_t index = i / 16;
          if (i < 256)
              Serial.write(' ');
          Serial.print(index, HEX);
          Serial.write('x');
          Serial.print(s);
          Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
        }
    }
    Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
}
void recv_cb(rui_lora_p2p_recv_t data) {
  rx_done = true;
  char buff[92];
  uint8_t respone = data.Buffer[10];
  Serial.println(respone,HEX);
  if (respone == 0x00 || 0x01 && data.Buffer[4]==0x01 && data.Buffer[5]==0x02 && data.Buffer[6]==0x03 && data.Buffer[7]==0x04 && data.Buffer[8]==0x06 && data.Buffer[9]==0x06 ) {
    join_success = true;
  } else{
    join_success = false;
  }
  if(data.Buffer[2]==0x01 && data.Buffer[3]==0x06 && data.Buffer[4]==0x01 && data.Buffer[5]==0x02 && data.Buffer[6]==0x03 && data.Buffer[7]==0x04 && data.Buffer[8]==0x06 && data.Buffer[9]==0x06 ){
    if(data.Buffer[10]==0x01){
        digitalWrite(LED_SYNC,1);
        Serial.println(data.Buffer[10]);
    }else if(data.Buffer[10]==0x00){
      Serial.println(data.Buffer[10]);
      digitalWrite(LED_SYNC,0);
    }
  }
  if(data.Status==1){
    digitalWrite(LED_RECV,0);
    Serial.println("________________ Recv Nothing ____________");
    digitalWrite(LED_RECV,0);
    return;
  }
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d", data.BufferSize, data.Rssi, data.Snr);
  digitalWrite(LED_RECV,0);
  // hexDump(data.Buffer, data.BufferSize);
  // sleep_mode();
}
void send_cb(void) {
  digitalWrite(LED_RECV,1); 
  digitalWrite(LED_SEND,0);
  Serial.printf("P2P Set RX Mode: %s\r\n", api.lora.precv(20000) ? "Success" : "Fail");  // bật mode RX 20s
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
  digitalWrite(LED_SEND,1);
  bool send_result = false;
  len = 0;                              //must have for reset len each call function
  frame[len++] = 0xFE;                  // BEGIN
  frame[len++] = 0;                     // LEN (placeholder)
  frame[len++] = frame_type;            // FRAME TYPE
  frame[len++] = function;              // FUNCTION
  memcpy(&frame[len], MAC, 6);          // MAC
  len += 6;                             //Mac address 6 bytes 
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
void sleep_mode(){
  Serial.println("________________  Sleeping ____________________-");
  digitalWrite(LED_SYNC,0);
  digitalWrite(LED_SEND,0);
  digitalWrite(LED_RECV,0);
  api.system.sleep.all();
  api.system.scheduler.task.destroy();
}