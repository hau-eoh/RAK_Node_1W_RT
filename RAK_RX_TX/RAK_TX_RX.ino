#define LED_RECV PB2
bool rx_done=false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
unsigned long startTime;
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
  // init_io();
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
  uint8_t payload[] = "NhinAnCutA";

  int rxDelay = 5000;

  // Receive P2P data every 10 seconds
  if(millis() - startTime >= 10*1000) {
    Serial.printf("P2P Rx start for %d millisSeconds\r\n", rxDelay);
    startTime = millis();
    Serial.printf("P2P set Rx mode %s\r\n",api.lora.precv(rxDelay) ? "Success" : "Fail");
    delay(rxDelay);
  } else {

    Serial.printf("P2P send %s\r\n", api.lora.psend(sizeof(payload), payload)? "Success" : "Fail");
    delay(5000);
  }
}

