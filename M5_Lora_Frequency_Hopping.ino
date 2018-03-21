// Adapted for the M5Stack from
// https://teknokoodiradio.vuodatus.net/lue/2017/07/sx1278-fast-frequency-hopping
// and
// https://github.com/open-electronics/LoRa/releases

#include <M5Stack.h>
#include <M5LoRa.h>

M5LineGraph lg;

#define LORA_CS_PIN   5
#define LORA_RST_PIN  26
#define LORA_IRQ_PIN  36

#define SERIAL_SPEED        115200
#define F_START             433 // start freq
#define F_STEP              0.1 // 25 kHz step
#define F_CH                920 // channels to rx hop
#define BPS                 1200
#define STATE_STANDBY       1
#define STATE_FSRX          4
#define STATE_RX            5
#define H_DELAY             2
#define M_DELAY             10
#define L_DELAY             100
#define SX_BEGIN            "LoRa.begin"
#define SX_BITRATE          "LoRa.readBPS "
#define SX_FREQ             "LoRa.readFreq "
#define RSSI_THRESHOLD      -130 // dBm
#define CH_FILTER_BW        B00001100 // 25 kHz

#define RegRssiConfig       0x0e
#define RegRxBw             0x12
#define RegPacketConfig1    0x30

#define RegOpMode           0x01
#define RegPaRamp           0x0a
#define RegRssiThresh       0x10
#define RegOokPeak          0x14
#define RegPreambleDetect   0x1f
#define RegSyncConfig       0x27
#define RegPacketConfig2    0x31
#define RegIrqFlags1        0x3e
#define RegIrqFlags2        0x3f
#define RegPllHop           0x44

bool rx = false;
uint16_t lineCount = 0;

#define SLEEP 0x00 // Lowest power. It allows change OOK/FSK to LoRa
#define RegFrfMsb 0x06
#define RegFrfMid 0x07
#define RegFrfLsb 0x08
#define RegBitrateMsb  0x02
#define RegBitrateLsb  0x03

void startModeFSKOOK() {
  setState(SLEEP); //sleep mode
  byte b = SPIread(RegOpMode);
  bitClear(b, 7);
  SPIwrite(RegOpMode, b);
}

void setState(byte opstate) {
  byte b = SPIread(RegOpMode);
  b = setBit(b, opstate, 0, 3);
  SPIwrite(RegOpMode, b);
}

int SPIread(byte address) {
  digitalWrite(LORA_CS_PIN, 0);
  SPI.transfer(address);
  delayMicroseconds(100);
  int val = SPI.transfer(0x00);
  digitalWrite(LORA_CS_PIN, 1);
  return val;
}

int SPIwrite(byte address, byte val) {
  digitalWrite(LORA_CS_PIN, 0);
  SPI.transfer(address | 0x80);
  delayMicroseconds(100);
  SPI.transfer(val);
  digitalWrite(LORA_CS_PIN, 1);
}

byte setBit(byte b, byte val, byte bst, byte len) {
  int i;
  for (i = 0; i < len; i++) bitWrite(b, i + bst, bitRead(val, i));
  return b;
}

void setFreq(float freq) {
  union Bfreq {
    unsigned long ifreq;
    unsigned char vfreq[4];
  } bf;
  bf.ifreq = (freq * 1000000) / 61.035;
  SPIwrite(RegFrfMsb, bf.vfreq[2]);
  SPIwrite(RegFrfMid, bf.vfreq[1]);
  SPIwrite(RegFrfLsb, bf.vfreq[0]);
}

float readFreq() {
  unsigned char f0 = SPIread(RegFrfLsb);
  unsigned char f1 = SPIread(RegFrfMid);
  unsigned char f2 = SPIread(RegFrfMsb);
  unsigned long lfreq = f2; lfreq = (lfreq << 8) + f1; lfreq = (lfreq << 8) + f0;
  float freq = (lfreq * 61.035) / 1000000;
  return freq;
}

void setBPS(int bps) {
  unsigned long lrate = 32000000 / bps;
  unsigned int rate = lrate;
  SPIwrite(RegBitrateMsb, highByte(rate));
  SPIwrite(RegBitrateLsb, lowByte(rate));
}

unsigned int readBPS() {
  unsigned char b0 = SPIread(RegBitrateLsb);
  unsigned char b1 = SPIread(RegBitrateMsb);
  unsigned int bps = word(b1, b0);
  unsigned long lbps = 32000000 / bps;
  bps = lbps;
  return bps;
}

float readRSSIval() {
  byte b = SPIread(0x11);
  float v = b;
  return -v / 2;
}

#define FSSB12 &FreeSansBold12pt7b

void setup() {
  Serial.begin(SERIAL_SPEED);
  M5.begin();
  // Lcd display
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setFont(FSSB12);
  Serial.print(F("\n\n\nM5 init "));

  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN); // set CS, reset, IRQ pin
  if (!LoRa.begin(F_START * 1E6)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
  Serial.println(F(SX_BEGIN));
  delay(M_DELAY);
  startModeFSKOOK();
  // OOK
  byte b = SPIread(RegOpMode);
  bitClear(b, 6);
  bitSet(b, 5);
  SPIwrite(RegOpMode, b);

  fastHop(0);
  setFreq(F_START);
  Serial.print(F(SX_FREQ));
  Serial.println(readFreq(), DEC);
  setBPS(BPS);
  Serial.print(F(SX_BITRATE));
  Serial.println(readBPS(), DEC);

  // Channel filter bandwidth control
  SPIwrite(RegRxBw, CH_FILTER_BW);

  // Bits 7-6: AutoRestartRxMode, 01 -> On, without waiting for the PLL to re-lock
  // Bit 4: Enables the Sync word generation and detection: 0 -> Off, 1 -> On
  // Bit 5: Sets the polarity of the Preamble. 0 -> 0xAA, 1 -> 0x55
  // Bits 2-0: Size of the Sync word (SyncSize + 1)
  b = SPIread(RegSyncConfig);
  bitClear(b, 7);
  bitSet(b, 6);
  bitClear(b, 4);
  bitClear(b, 5);
  bitClear(b, 2);
  bitClear(b, 1);
  bitClear(b, 0);
  SPIwrite(RegSyncConfig, b);

  // Bits 6-5: Defines DC-free encoding/decoding performed:
  // 00 -> none, 01 -> manchester, 10 -> whitening
  // Bit 7: packet format, 0 -> fixed length, 1 -> variable length
  // Bit 4: crc calc/check, 0 -> off, 1 -> on
  // Bits 2-1: Defines address based filtering in Rx: 00 ->Â None (Off)
  // Bit 3: Defines the behavior of the packet handler when CRC check fails:
  // 0 -> Clear FIFO and restart new packet reception. No PayloadReady interrupt issued.
  // 1 -> Do not clear FIFO. PayloadReady interrupt issued.
  // Bit 0: Selects the CRC and whitening algorithms:
  // 0 -> CCITT CRC implementation with standard whitening
  // 1 -> IBM CRC implementation with alternate whitening
  b = 0;
  SPIwrite(RegPacketConfig1, b);

  // Bits 6-5: FSK data shaping:
  // 00 -> no shaping, 01 -> Gaussian filter BT = 1.0
  // 10 -> Gaussian filter BT = 0.5, 11 -> Gaussian filter BT = 0.3
  // Bits 3-0: Rise/Fall time of ramp up/down in FSK:
  // 1001 -> 40 us
  b = SPIread(RegPaRamp);
  bitClear(b, 6);
  bitClear(b, 5);
  bitSet(b, 3);
  bitClear(b, 2);
  bitSet(b, 0);
  SPIwrite(RegPaRamp, b);

  // Data processing mode: 0 ->Â Continuous mode, 1 -> Packet mode
  b = SPIread(RegPacketConfig2);
  bitClear(b, 6);
  SPIwrite(RegPacketConfig2, b);

  // RSSI smoothing.
  // Defines the number of samples taken to average the RSSI result. 001 -> 4 samples
  b = SPIread(RegRssiConfig);
  bitClear(b, 2);
  bitClear(b, 1);
  bitSet(b, 0);
  SPIwrite(RegRssiConfig, b);
  SPIwrite(RegRssiThresh, RSSI_THRESHOLD);

  // Bit 5: enables the Bit Synchronizer:
  // 0 -> bit sync disabled (not possible in packet mode), 1 -> bit sync enabled
  b = SPIread(RegOokPeak);
  bitClear(b, 5);
  SPIwrite(RegOokPeak, b);

  // RegPreambleDetect (0x1f). Enables Preamble detector when set to 1.
  // The AGC settings supersede this bit during the startup / AGC phase.
  // Bit 7: 0 -> turned off, 1 -> turned on
  // Bits 6-5: Number of Preamble bytes to detect to trigger an interrupt.
  // 00 -> 1 byte, 10 -> 3 bytes, 01 -> 2 bytes
  b = SPIread(RegPreambleDetect);
  bitClear(b, 7);
  bitClear(b, 6);
  bitClear(b, 5);
  SPIwrite(RegPreambleDetect, b);
  clearFifoAndFlags();
  setState(STATE_FSRX);
  delay(L_DELAY);
  setState(STATE_RX);
  delay(M_DELAY);
  rx = true;

  lg.setAutoClear(true);
  lg.setBounds(0, 0, 320, 200);
  lg.setLineColor(0, TFT_BLUE);
  lg.setLineCount(1);
  lg.setRange(RSSI_THRESHOLD, -20);
  lg.draw();
}

void loop() {
  if (rx) {
    float f = F_START;
    for (int c = 0; c < F_CH; c++) {
      fastHop(1);
      setFreq(f);
      delay(H_DELAY);
      float rssi = readRSSIval();
      Serial.print(F("Freq: "));
      Serial.print(f);
      Serial.print(F(" RSSI: "));
      Serial.println(rssi);

      M5.Lcd.fillRect(0, 201, 320, 40, TFT_BLACK);
      M5.Lcd.drawString("Freq: " + String(f) + " RSSI: " + String(rssi), 10, 210, 1);
      if (lineCount > 320) {
        lg.clear();
        lg.undraw();
        lg.draw();
        lineCount = 0;
      }
      lg.addValue(rssi);
      lg.draw();
      lineCount++;
      Serial.print(f, DEC);
      Serial.print("\t");
      Serial.println(rssi, DEC);
      if (rssi > -118) delay(1000);
      f = f + F_STEP;
      delay(L_DELAY);
    }
  }
}

void clearFifoAndFlags() {
  // Flag(s) and FIFO are cleared when this bit is set
  byte b = SPIread(RegIrqFlags2);
  bitSet(b, 4);
  SPIwrite(RegIrqFlags2, b);
  delay(M_DELAY);
}

void fastHop(int val) {
  byte b = SPIread(RegPllHop);
  bitWrite(b, 7, val);
  SPIwrite(RegPllHop, b);
}

