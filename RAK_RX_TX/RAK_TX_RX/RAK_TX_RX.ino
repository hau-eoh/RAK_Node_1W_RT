

bool rx_done=false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
unsigned long startTime;
#define ONEWIRE_PIN_DATA GPIO_PIN_12
#define ONEWIRE_PORT_DATA GPIOA
void delay_us(uint8_t time){
  unsigned long pre=0;
  while(micros()-pre >= time){
    // Nếu micros 
    pre = micros();
  }
}
void set_pin_as_output(GPIO_TypeDef *port, uint16_t pin) {
  // Cấu hình chân GPIO thành chế độ xuất (output).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}
void set_pin_as_input(GPIO_TypeDef *port, uint16_t pin) {
    // Cấu hình chân GPIO thành chế độ nhập (input).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}
void onewire_reset(void) {
    // Đặt lại cảm biến DS18B20.
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ xuất.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);  // Kéo chân dữ liệu xuống mức thấp.
    delay_us(480);  // Chờ 480 microseconds.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);  // Kéo chân dữ liệu lên mức cao.
    delay_us(480);  // Chờ thêm 480 microseconds.
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ nhập.
    delay_us(480);  // Chờ thêm 480 microseconds.
}
// void onewire_write(uint8_t data) {
//   for(uint8_t i=0;i<8;i++)
//   {
//     set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
//     if(data&(1<<i)==HIGH){
//         HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
//         delay_us(1);
//         HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
//         delay_us(60);
//     }else{
//         HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
//         delay_us(60);
//         HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
//         delay_us(1);
//     }
//     set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
//   }
// }
void onewire_write_bit(uint8_t bit) {
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    if (bit) { //Nếu bit la 1
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delay_us(1);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
        delay_us(60);
    } else {
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delay_us(60);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
        delay_us(1);
    }
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
}
void onewire_write_byte(uint8_t data) {
    // Ghi một byte vào DS18B20.
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(data & 0x01);  // Ghi bit thấp nhất của byte.
        data >>= 1;  // Dịch chuyển byte sang phải 1 bit.
    }
}
// uint8_t onewire_read(void) {
//   uint8_t value=0;
//   for(uint8_t i=0; i<8;i++){
//     set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
//     HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
//     delay_us(1);
//     HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
//     set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
//     if(HAL_GPIO_ReadPin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA==HIGH))
//     {
//       value |= 1<<i;
//     }else{
//       value |= 0<<i;  
//     }
//     delay_us(60);
//     return value;
//   }
// }
uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    delay_us(15);
    bit = HAL_GPIO_ReadPin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    delay_us(45);
    return bit;
}

// Ghi 1 byte vào DS18B20
uint8_t onewire_read_byte(void) {
    // Đọc một byte từ DS18B20.
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        data |= (onewire_read_bit() << i);  // Đọc từng bit và dịch chuyển vào byte.
    }
    return data;  // Trả về giá trị byte đọc được.
}
float read_temperature(void) {
    uint8_t temp_LSB, temp_MSB;
    int16_t temp;

    // Khởi động lại cảm biến
    onewire_reset();

    // Gửi lệnh Skip ROM
    onewire_write_byte(0xCC);

    // Gửi lệnh Convert T
    onewire_write_byte(0x44);

    // Chờ cảm biến hoàn thành quá trình chuyển đổi nhiệt độ
    delay(750);

    // Khởi động lại cảm biến
    onewire_reset();

    // Gửi lệnh Skip ROM
    onewire_write_byte(0xCC);

    // Gửi lệnh Read Scratchpad
    onewire_write_byte(0xBE);

    // Đọc 2 byte dữ liệu nhiệt độ
    temp_LSB = onewire_read_byte();
    temp_MSB = onewire_read_byte();

    // Chuyển đổi dữ liệu nhiệt độ
    temp = ((int16_t)temp_MSB << 8) | temp_LSB;

    // Trả về giá trị nhiệt độ dạng float
    return (float)temp / 16.0;
}
void recv_cb(rui_lora_p2p_recv_t data)
{
  rx_done = true;
  if (data.BufferSize == 0)
  {
    Serial.println("Empty buffer.");
    return;
  }
  digitalWrite(LED_RECV, HIGH);
  char buff[92];
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d",
          data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);
  digitalWrite(LED_RECV, LOW);
}

void send_cb(void)
{
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
    // Serial.println("I send something");
}
void setup()
{
  Serial.begin(115200);
  Serial.println("RAK3172 TEST");
  Serial.println("------------------------------------------------------");
  init_io();
  enable_Vss5();
  delay(1000);/////////////////////// Làm Ơn Đừng Xóa Cái Dòng Này An À /////////////////////////////////////////////////////
  // startTime = millis();

  // if (api.lora.nwm.get() != 0)// Set P2P mode
  // {
  //   Serial.printf("Set Node device work mode %s\r\n",api.lora.nwm.set() ? "Success" : "Fail");
  //   api.system.reboot();
  // }
  // Serial.println("P2P Start");
  // Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());// ID chip
  // Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());//ID model
  // Serial.printf("RUI API Version: %s\r\n",api.system.apiVersion.get().c_str());//Ver API
  // Serial.printf("Firmware Version: %s\r\n",api.system.firmwareVersion.get().c_str());//Ver Firmware
  // Serial.printf("AT Command Version: %s\r\n",api.system.cliVersion.get().c_str());//Ver AT command
  // Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6),api.lora.pfreq.set(myFreq) ? "Success" : "Fail");//set tần số
  // Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf,api.lora.psf.set(sf) ? "Success" : "Fail");//set spreading factor
  // Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw,api.lora.pbw.set(bw) ? "Success" : "Fail");//set băng thông
  // Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5),api.lora.pcr.set(cr) ? "Success" : "Fail");//set code  rate
  // Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble,api.lora.ppl.set(preamble) ? "Success" : "Fail");//set preamble length
  // Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower,api.lora.ptp.set(txPower) ? "Success" : "Fail");//set TX power
  // api.lora.registerPRecvCallback(recv_cb);// goij hàm recv_cb mỗi khi nhận dữ liệu xong
  // api.lora.registerPSendCallback(send_cb);// gọi hàm send_cb mỗi khi gửi xong
  // Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(65533) ? "Success" : "Fail");//Set RX mode
  /*

  timeout - the duration of P2P Rx mode in milliseconds
Special timeout values:
65535: Stay in RX mode until a packet is received.
65534: Permanent RX mode (TX not possible) until api.lora.precv(0) is executed.
65533: Permanent RX mode and still allows TX without calling api.lora.precv(0).
  */

}

void loop()
{
  // uint8_t payload[] = "NhinAnCutA";

  // int rxDelay = 5000;

  // // Receive P2P data every 10 seconds
  // if(millis() - startTime >= 10*1000) {
  //   Serial.printf("P2P Rx start for %d millisSeconds\r\n", rxDelay);
  //   startTime = millis();
  //   Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(rxDelay) ? "Success" : "Fail");
  //   delay(rxDelay);
  // } else {

  //   Serial.printf("P2P send %s\r\n", api.lora.psend(sizeof(payload), payload)? "Success" : "Fail");
  //   delay(5000);
  // }
  float temperature = read_temperature();
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);  
  Serial.println(" *C");
  delay(1000); 
}

