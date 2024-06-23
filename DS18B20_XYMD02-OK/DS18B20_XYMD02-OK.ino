#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
#include "XY_MD02.h"
#include "ds18b20.h"
uint32_t MAC =0x240602;
uint16_t temp;
uint8_t frame[11];// tạo mảng 9 ô nhớ, mỗi ô chứa 8bit
bool rx_done=false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
unsigned long startTime;
#define ONEWIRE_PIN_DATA GPIO_PIN_12
#define ONEWIRE_PORT_DATA GPIOA
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
void setup()
{
  Serial.begin(115200);
  Serial_Canopus.begin(9600, SERIAL_8N1);
  Serial.println("RAK3172 TEST");
  Serial.println("------------------------------------------------------");
  /////////////////////// Delay For Detect baudrate- DO NOT DELETE /////////////////////////////////////////////////////
  delay(1000);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  init_io();
  enable_Vss5();
  enable_Vrs485();
  init_xy();
  // node.begin(1, Serial_Canopus);

  startTime = millis();

  if (api.lora.nwm.get() != 0)// Set P2P mode
  {
    Serial.printf("Set Node device work mode %s\r\n",api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }
  Serial.println("P2P Start");
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());// ID chip
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());//ID model
  Serial.printf("RUI API Version: %s\r\n",api.system.apiVersion.get().c_str());//Ver API
  Serial.printf("Firmware Version: %s\r\n",api.system.firmwareVersion.get().c_str());//Ver Firmware
  Serial.printf("AT Command Version: %s\r\n",api.system.cliVersion.get().c_str());//Ver AT command
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6),api.lora.pfreq.set(myFreq) ? "Success" : "Fail");//set tần số
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf,api.lora.psf.set(sf) ? "Success" : "Fail");//set spreading factor
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw,api.lora.pbw.set(bw) ? "Success" : "Fail");//set băng thông
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5),api.lora.pcr.set(cr) ? "Success" : "Fail");//set code  rate
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble,api.lora.ppl.set(preamble) ? "Success" : "Fail");//set preamble length
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower,api.lora.ptp.set(txPower) ? "Success" : "Fail");//set TX power
  api.lora.registerPRecvCallback(recv_cb);// goij hàm recv_cb mỗi khi nhận dữ liệu xong
  api.lora.registerPSendCallback(send_cb);// gọi hàm send_cb mỗi khi gửi xong
  Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(65533) ? "Success" : "Fail");//Set RX mode
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
  temp = read_temperature();
  long currentMillis = millis();
  uint16_t xy_md02_temp = read_temp();
  uint16_t xy_md02_humi = read_humi();
  frame[0]=0xFE;//  begin 1 ô
  frame[1]=0x08;//len 1 ô
  frame[2]=0x02;// frame type 1 ô 
  frame[3]=0x04;// funtion 1 ô
  frame[4]=MAC>>16;
  frame[5]=MAC>>8;
  frame[6]=MAC;
    // memcpy(&frame[4], MAC, 3);//3 ô
  frame[7]=xy_md02_temp>>8;
  frame[8]=xy_md02_temp;
  frame[9]=0xAA;// 2 ô 
  frame[10]=0xEF;// 2 ô
  Serial.println("Frame Truyền: ");
  for(int i=1;i<=3;i++)
  {
    switch(i){
      case 1:
        Serial.println("DATA: XY-MD02 Humi:");
        frame[3]=0x02;
        frame[7]=xy_md02_humi>>8;
        frame[8]=xy_md02_humi;
        break;
      case 2:
        Serial.println("DATA: XY-MD02 Temp:");
        frame[3]=0x03;
        frame[7]=xy_md02_temp>>8;
        frame[8]=xy_md02_temp;
        break;
      case 3:
        Serial.println("DATA: DS18B20:");
        frame[3]=0x04;
        frame[7]=temp>>8;
        frame[8]=temp;
      break;
    }
    for(int i=0;i<sizeof(frame);i++){
      Serial.printf("%X",frame[i]);
    }
    Serial.println("");
  }
  Serial.println("");
  Serial.print("Temperature-DS18B20: ");
  Serial.print((float)temp/16.0);
  Serial.print("*C");
  Serial.printf(" -- %X\n",temp);
  Serial.print("XY-Temp: ");
  Serial.print(xy_md02_temp/10.0);
  Serial.printf(" -- %X\n",xy_md02_temp);
  Serial.print("XY-Humi: ");
  Serial.print(xy_md02_humi/10.0);
  Serial.printf(" -- %X\n",xy_md02_humi);
  delay(1000); 
  // int rxDelay = 5000;
  // // Receive P2P data every 10 seconds
  // if(millis() - startTime >= 10*1000)
  // {
  //   Serial.printf("P2P Rx start for %d millisSeconds\r\n", rxDelay);
  //   startTime = millis();
  //   Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(rxDelay) ? "Success" : "Fail");
  //   delay(rxDelay);
  // } 
  // else
  // {
  //   Serial.printf("P2P send %s\r\n", api.lora.psend(sizeof(frame),frame)? "Success" : "Fail");
  //   delay(5000);
  // }

}


