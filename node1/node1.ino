#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
#include "ds18b20.h"
#define button_pin PA11
ModbusMaster node;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
uint8_t MAC[6] = { 0x0A, 0x0B, 0x0B, 0x0C,0x0D, 0x0E };
uint8_t frame[16];
bool join_success = false;
bool rx_done=false;
bool WakeUpFlag=false;
uint16_t xy_md02_humi ;  // biến độ ẩm từ rs485
uint16_t xy_md02_temp ;       // biến nhiệt độ từ rs485 16bit
unsigned long lastMillis = 0;
int count=0;
long currentMillis = millis();
long lastSent=0;
static unsigned long markButtonClick=0;
void send_cb(void) {
  digitalWrite(LED_RECV,1); 
  digitalWrite(LED_SEND,0);
  api.lora.precv(0);
  Serial.printf("P2P Set RX Mode: %s\r\n", api.lora.precv(65534) ? "Success" : "Fail");  // bật mode RX 20s
}
void recv_cb(rui_lora_p2p_recv_t data) {
  rx_done = true;
  uint8_t response = data.Buffer[10];
  // char buff[92];
  // sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d", data.BufferSize, data.Rssi, data.Snr);
  // Serial.println(buff);
  if (response == 0x00 ||response ==  0x01 && memcmp(&data.Buffer[4],MAC,6)==0 && join_success==false) {
    join_success = true;
  } else if(response == 0x02 && join_success==false){
    join_success = false;// nếu buff[5] k phải 0x00 hoặc 0x01 thì thoát khỏi hàm 
    return;
  }
  if(data.Buffer[2]==0x01 && data.Buffer[3]==0x06 && memcmp(&data.Buffer[4],MAC,6)==0){// nếu buff[2](funtion)=0x01 và frametype =0x06 
    if(data.Buffer[10]==0x01){//kiểm tra frame data
        digitalWrite(LED_SYNC,1);//bật led sync
        Serial.println(data.Buffer[10]);
    }else if(data.Buffer[10]==0x00){
      Serial.println(data.Buffer[10]);
      digitalWrite(LED_SYNC,0);//tắt led sync
    }
  }
  if(data.Status==1){//Nếu RX k nhận đc gì sau 20s thì sẽ sleep(thời gian time out đc set trong hàm send_cb)
    digitalWrite(LED_RECV,0);
    Serial.println("________________ Recv Nothing ____________");
    digitalWrite(LED_RECV,0);
    return;
  }
  digitalWrite(LED_RECV,0);
  // hexDump(data.Buffer, data.BufferSize);
}
void hexDump(uint8_t * buf, uint16_t len)
{
    char alphabet[17] = "0123456789abcdef";
    Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
    Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\r\n"));
    for (uint16_t i = 0; i < len; i += 16) {
        if (i % 128 == 0)
            Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
        char s[] = "|                                                | |                |\r\n";
        uint8_t ix = 1, iy = 52;
        for (uint8_t j = 0; j < 16; j++) {
            if (i + j < len) {
  	            uint8_t c = buf[i + j];
  	            s[ix++] = alphabet[(c >> 4) & 0x0F];
  	            s[ix++] = alphabet[c & 0x0F];
  	            ix++;
  	            if (c > 31 && c < 128)
  	                s[iy++] = c;
  	            else
  	                s[iy++] = '.';
            }
        }
        uint8_t index = i / 16;
        if (i < 256)
            Serial.write(' ');
        Serial.print(index, HEX);
        Serial.write('.');
        Serial.print(s);
    }
    Serial.print(F("   +------------------------------------------------+ +----------------+\r\n"));
}
uint8_t crc8(const uint8_t *data, int len) 
{
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
  digitalWrite(LED_SEND,1);// bật led send mỗi khi gửi
  bool send_result = false;
  uint8_t len = 0;                              //must have for reset len each call function
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
      delay(500);
    }
    //Check số lần gửi
    retry_count++;  // Tăng số lần thử gửi lại.
    Serial.print("Check resend: ");
    Serial.println(retry_count);
  }
  // In frame đã truyền
  Serial.print("Sending frame: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}
void WakeUp()// mỗi khi thức dậy sẽ chạy hàm này
{   
  Serial.println("Hàm Callback ");
  OneWire_frame();
  delay(10000);
  humi_frame();
  delay(10000);
  temp_frame();
}

void OneWire_frame(){
  float temp = read_temperature();
  uint16_t temp_scaled = temp * 10;          //123
  uint8_t temp_data[2];                   // mảng tách giá trị của nhiệt độ từ 1W
  Serial.print("scaled temp: ");
  Serial.println(temp_scaled);
  Serial.print("temp: ");
  Serial.println(temp);
  temp_data[0] = temp_scaled & 0xFF;         // LSB//8bit cao
  temp_data[1] = (temp_scaled >> 8) & 0xFF;
  send_frame(0x03,0x04,temp_data,2);
}
void humi_frame(){
  uint8_t humi_data[2];                 // mảng tách giá trị độ ẩm từ rs485
  humi_data[0] = xy_md02_humi;
  humi_data[1] = xy_md02_humi >> 8;
  send_frame(0x03, 0x02, humi_data, 2); 
}
void temp_frame(){
  
  uint8_t tempmd02_data[2];                  // mảng tách giá trị nhiệt độ từ rs485
  tempmd02_data[0] = xy_md02_temp;
  tempmd02_data[1] = (xy_md02_temp >> 8);
  send_frame(0x03, 0x03, tempmd02_data, 2);
}
void read_rs485()
{
  //***************READ node 1**************************
   // slave ID node
  long currentMillis = millis();
  if (currentMillis - lastMillis > 1000) {
    uint8_t result = node.readInputRegisters(0x01, 2);
    if (getResultMsg(&node, result)) {
      Serial.println();
      float res_dbl = node.getResponseBuffer(0) / 10.0;
      xy_md02_temp = node.getResponseBuffer(0);
      // Serial.println(node.getResponseBuffer(0),HEX);
      String res = "Temperature: " + String(res_dbl) + " C\r\n";
      res_dbl = node.getResponseBuffer(1) / 10.0;
      xy_md02_humi = node.getResponseBuffer(1);
      // Serial.println(node.getResponseBuffer(1),HEX);
      res += "Humidity: " + String(res_dbl) + " %";
      Serial.println(res);
    }
    lastMillis = currentMillis;
  }
}
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
void adam_frame(uint8_t status_DI){
  uint8_t adam_data[4];
  adam_data[0] = 0;
  adam_data[1] = status_DI;
  adam_data[2] = 0;
  adam_data[3] = 0;
  send_frame(0x03,0x05,adam_data,4);
}
void batt_frame(){
  uint8_t battery=(int)((api.system.bat.get()/4.2)*100);
  Serial.printf("Battery: %d%% ",battery);
  send_frame(0x03,0x07,&battery,1);
}
void itrButton(){
  ButtonKick();
}
void ButtonKick(){
  if(digitalRead(button_pin)==LOW && millis()-markButtonClick>=20){
    Serial.println("_______________________________  External Interrupt!!!_________________________________________");
    adam_frame(1);
    delay_ms(6000);
    adam_frame(0);
  }else if(digitalRead(button_pin)==HIGH){
    markButtonClick=millis();
  }
}
void delay_ms(int number_of_milliseconds) {
    unsigned long start_time = millis();
    unsigned long end_time = start_time + number_of_milliseconds;

    while (millis() < end_time) {
        // Keep looping until the specified time has passed
        // This avoids blocking the main loop
    }
}
void setup() {
  Serial.begin(115200);
  Serial_Canopus.begin(9600, SERIAL_8N1);
  Serial.println(" RAK3172 TEST");
  Serial.println("------------------------------------------------------");
  /////////////////////// Delay For Detect baudrate- DO NOT DELETE ////////
  delay(2000);
  ////////////////////////////////////////////////////////////////////////
  init_io();        // kích hoạt chân gpio,led
  enable_Vss3();    // Init nguồn 5v
  enable_Vrs485();  // Init cổng rs485
  node.begin(1, Serial_Canopus);
  // init_xymd02();    // init XY_MD02
  // startTime = millis();
  // pinMode(button_pin,INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(button_pin), itrButton, CHANGE);
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
  // Serial.printf("Set the low power mode %s\n\r", api.system.lpm.set(1) ? "Success" : "Fail");  //set Low Power mode
  api.lora.registerPRecvCallback(recv_cb);  // gọi hàm recv_cb mỗi khi vào mode RX
  api.lora.registerPSendCallback(send_cb);// gọi hàm send_cb mỗi khi gửi xong
  Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(3000) ? "Success" : "Fail");
  for(int i=0;i<20;i++){
    digitalWrite(LED_SYNC,!digitalRead(LED_SYNC));
    delay(50);
  }
  digitalWrite(LED_SYNC,1);
  while (!join_success) {
    uint8_t join_data[1] = { 0x01 };       // node dùng cảm biến rời + onewire
    send_frame(0x01, 0x01, join_data, 1);  // gửi frame request join lên gateway
    Serial.println("Join request Sent");
    delay(5000);
    if(join_success){
      Serial.println("-----------------------               --------------------");
      Serial.println("----------------------- Join Success --------------------");
      Serial.println("-----------------------               --------------------");
      adam_frame(0);
      digitalWrite(LED_SYNC,0);
      break;
      // battery = (api.system.bat.get()/4.2)*100;
      // batt_int = battery;
      // send_frame(0x03, 0x07, &batt_int, 1);//gửi dung lượng pin lần đầu
      // sensor_frame();//gửi data sensor lần đầu
    }else{
      Serial.println("-----------------------               --------------------");
      Serial.println("------------------- !!! JOIN FAIL !!!! -------------------");
      Serial.println("-----------------------               --------------------");
    }
    delay(10000);
  }
  if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)WakeUp, RAK_TIMER_PERIODIC) != true) {
    Serial.printf("Creating timer failed.\r\n");
  } 
  else if (api.system.timer.start(RAK_TIMER_0, 60000, (void*)0) != true) {//Setup thời gian thức dậy(Ngủ 1p--> Dậy gửi data và ngủ lại)
    Serial.printf("Starting timer failed.\r\n");
  }
  pinMode(button_pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), itrButton, CHANGE);
}
void loop() {
  // put your main code here, to run repeatedly:
  read_rs485();
  // delay(30000);
  // OneWire_frame();
  // delay(5000);
  // humi_frame();
  // delay(5000);
  // temp_frame();
  // delay(5000);
}
