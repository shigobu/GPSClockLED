#include <SdFat.h>
#include <time.h>
#include <TinyGPS++.h>
#include <TM1637.h>
#include <TM16xxDisplay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#define RTC_PERIOD (511)

#include "unions.h"
#include "GPSClockLED.h"

TinyGPSPlus gps;

SdFat32 sdFat;
char tzid[40] = { 'U', 'T', 'C' };
int16_t currentOffsetMinutes = 0;

TM1637 sevenSegment(SEVEN_SEG_DIO_PIN, SEVEN_SEG_CLK_PIN);    //  DIO, CLK
TM16xxDisplay display(&sevenSegment, 6);    // TM16xx object, 6 digits

byte timeVectCount = 0;   // RTC割り込みは、500ms毎にしているので、2回毎に一秒進める必要があるので、それのカウント用。

bool isSecondDisp = true;
bool isBright = false;
static byte colLedHighVal = 100;
static byte colLedLowVal = 10;

void setup()
{
  pinMode(ONE_PPS_PIN, INPUT);
  pinMode(GPS_OFF_PIN, OUTPUT);
  pinMode(COL_LED_PIN1, OUTPUT);
  pinMode(COL_LED_PIN2, OUTPUT);
  GpsOn();
  display.setIntensity(1);

  //シリアルポート開始
  Serial.begin(9600);

  //タイマー割り込み設定
  RTC_init();

  //SD開始
  if (!sdFat.begin(SD_CS_PIN, SPI_FULL_SPEED))
  {
    digitalWrite(COL_LED_PIN1, HIGH);
    while (1)
    {
      ;
    }
  }

  delay(1000);

  /* 全体割り込み許可 */
  sei();

  //タイムゾーンの設定
  set_zone(0);    //UTC
  set_dst(NULL);  //サマータイム設定無し
  set_system_time(0);

  //gps情報の送信レートを設定
  Serial.println(F("$PMTK314,5,1,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,5,0*29"));
  setSystemTimeFromGPS();
  setTimeZoneOffset();
  //set_zone関数の引数は秒
  set_zone(currentOffsetMinutes * 60);

  while (1)
  {
    ;
  }

}

void loop()
{
  //set_zone関数の引数は秒
  set_zone(currentOffsetMinutes * 60);

  //システム時間の更新
  setSystemTimeFromGPS();

  //タイムゾーンの設定

  //オフセットの手動設定
  //  swState = getIsUpSwitchPressed();
  //  if (swState == SwitchPressedState::ShortPressed) {
  //    currentOffsetMinutes += 15;
  //  }
  //  swState = getIsDownSwitchPressed();
  //  if (swState == SwitchPressedState::ShortPressed) {
  //    currentOffsetMinutes -= 15;
  //  }

}

// 7segLEDに現在時刻を表示します。
void displayTime()
{
  time_t nowTime = time(NULL);
  tm* ptimeStruct;
  ptimeStruct = localtime(&nowTime);
  int val = (ptimeStruct->tm_hour * 100) + ptimeStruct->tm_min;
  sevenSegment.setDisplayToDecNumber(val);
  if (isSecondDisp)
  {
    sevenSegment.setDisplayDigit(ptimeStruct->tm_sec / 10, 4);
    sevenSegment.setDisplayDigit(ptimeStruct->tm_sec % 10, 5);
  }
}

void RTC_init(void)
{
  uint8_t temp;
  /* 32.768kHz発振器初期化 */
  /* 発振器禁止 */
  temp = CLKCTRL.XOSC32KCTRLA;
  temp &= ~CLKCTRL_ENABLE_bm;
  /* 保護されたﾚｼﾞｽﾀへの書き込み */
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
  {
    ; /* XOSC32KSが0になるまで待機 */
  }
  /* SEL=0 (外部ｸﾘｽﾀﾙ使用) */
  temp = CLKCTRL.XOSC32KCTRLA;
  temp &= ~CLKCTRL_SEL_bm;
  /* 保護されたﾚｼﾞｽﾀへの書き込み */
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
  /* 発振器許可 */
  temp = CLKCTRL.XOSC32KCTRLA;
  temp |= CLKCTRL_ENABLE_bm;
  /* 保護されたﾚｼﾞｽﾀへの書き込み */
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
  /* RTC初期化 */
  while (RTC.STATUS > 0)
  {
    ; /* 全ﾚｼﾞｽﾀが同期されるまで待機 */
  }
  /* 周期設定 */
  RTC.PER = RTC_PERIOD;
  /* 32.768kHz外部ｸﾘｽﾀﾙ用発振器 (XOSC32K) */
  RTC.CLKSEL = 0b10;
  /* ﾃﾞﾊﾞｯｸﾞで走行: 許可 */
  RTC.DBGCTRL |= RTC_DBGRUN_bm;
  RTC.CTRLA = RTC_PRESCALER_DIV32_gc /* 32分周 */
              | RTC_RTCEN_bm /* 許可: 許可 */
              | RTC_RUNSTDBY_bm; /* ｽﾀﾝﾊﾞｲで走行: 許可 */
  /* 溢れ割り込み許可 */
  RTC.INTCTRL |= RTC_OVF_bm;
}

//RTC割り込み
ISR(RTC_CNT_vect)
{
  /* ‘1’書き込みによって割り込み要求ﾌﾗｸﾞ解除(0) */
  RTC.INTFLAGS = RTC_OVF_bm;

  if (timeVectCount == 0)
  {
    timeVectCount++;
    setColLED(false);
  }
  else
  {
    timeVectCount = 0;
    timeTick();
    setColLED(true);
  }
}

// システム時間を一秒すすめて、時間表示する
void timeTick()
{
  //システム時間を一秒すすめる。
  system_tick();

  //時間表示
  displayTime();
}

void setColLED(bool isOn)
{
  if (isOn)
  {
    if (isBright)
    {
      analogWrite(COL_LED_PIN1, colLedHighVal);
      analogWrite(COL_LED_PIN2, colLedHighVal);
    }
    else
    {
      analogWrite(COL_LED_PIN1, colLedLowVal);
      //analogWrite(COL_LED_PIN2, colLedLowVal);
    }
  }
  else
  {
    analogWrite(COL_LED_PIN1, 0);
    analogWrite(COL_LED_PIN2, 0);
  }
}

//GPSから時刻を取得し、システム時間を設定します。GPSが3次元測位するまでブロックします。
void setSystemTimeFromGPS()
{
  GpsOn();

  while (true)
  {
    if (Serial.available() == 0)
    {
      continue;
    }
    char c = Serial.read();
    gps.encode(c);
    if (gps.time.isValid() && gps.date.isValid() && (digitalRead(ONE_PPS_PIN) == LOW))
    {
      while (RTC.STATUS & RTC_CNTBUSY_bm)
      {
        ; /* CTRLABUSYﾚｼﾞｽﾀが同期されるまで待機 */
      }
      /* RTCカウンタ初期化。１PPSと同期させる */
      RTC.CNT = 0;
      timeVectCount = 0;
      
      struct tm gps_time;
      gps_time.tm_sec = gps.time.second();
      gps_time.tm_min = gps.time.minute();
      gps_time.tm_hour = gps.time.hour();
      gps_time.tm_mday = gps.date.day();
      gps_time.tm_wday = 0;
      gps_time.tm_mon = gps.date.month() - 1;     //tm構造体は0-11の範囲なので１引く
      gps_time.tm_year = gps.date.year() - 1900;  //tm構造体は1900年起点なので1900を引く
      gps_time.tm_yday = 0;
      gps_time.tm_isdst = 0;

      set_system_time(mk_gmtime(&gps_time));
      break;
    }
  }

  //GpsOff();
}

//gpsから現在地を取得し、タイムゾーンを検索してオフセットを設定します。
void setTimeZoneOffset()
{

  isSecondDisp = false;

  //進捗を00に設定
  sevenSegment.setDisplayDigit(0, 4);
  sevenSegment.setDisplayDigit(0, 5, true);

  //todo　gpsから現在地取得
  float x = 0;
  float y = 0;

  if (gps.location.isValid())
  {
    //x = gps.location.lat();
    //y = gps.location.lng();
    x = gps.location.lng();
    y = gps.location.lat();
  }
  else
  {
    goto ERR;
  }

  //ファイル開く
  File32 binFile = sdFat.open(F("DATA.bin"));
  if (!binFile)
  {
    goto ERR;
  }

  memset(tzid, 0, sizeof(tzid));
  int iTzid = 0;
  int cn = 0;
  float oldX = NAN;
  float oldY = NAN;
  SHORT_INSIDE offset;
  uint32_t fileSize = binFile.fileSize();
  while (binFile.available())
  {
    int readData = 0;
    while ((readData = binFile.read()) != -1)
    {
      char readChar = readData;
      tzid[iTzid] = readChar;
      iTzid++;
      //null文字がたら、tzid決定。
      if (readChar == '\0')
      {
        iTzid = 0;
        break;
      }
    }
    //オフセットを読む
    offset.byteData[0] = binFile.read();
    offset.byteData[1] = binFile.read();

    //tzid決定後は、座標データを読み込む。
    //読み込みバイト数
    INT_INSIDE intIn;
    intIn.byteData[0] = binFile.read();
    intIn.byteData[1] = binFile.read();
    intIn.byteData[2] = binFile.read();
    intIn.byteData[3] = binFile.read();

    FLOAT_INSIDE floatInsideX;
    FLOAT_INSIDE floatInsideY;
    for (int32_t i = 0; i < intIn.data; i += 8)
    {
      floatInsideX.byteData[0] = binFile.read();
      floatInsideX.byteData[1] = binFile.read();
      floatInsideX.byteData[2] = binFile.read();
      floatInsideX.byteData[3] = binFile.read();
      floatInsideY.byteData[0] = binFile.read();
      floatInsideY.byteData[1] = binFile.read();
      floatInsideY.byteData[2] = binFile.read();
      floatInsideY.byteData[3] = binFile.read();
      //一番はじめの座標決定
      if (oldX == NAN)
      {
        oldX = floatInsideX.data;
        oldY = floatInsideY.data;
        continue;
      }

      //https://www.nttpc.co.jp/technology/number_algorithm.html　からコピペ
      // 上向きの辺。点Pがy軸方向について、始点と終点の間にある。ただし、終点は含まない。(ルール1)
      // if (((oldY <= y) && (floatInsideY.data > y))
      //   // 下向きの辺。点Pがy軸方向について、始点と終点の間にある。ただし、始点は含まない。(ルール2)
      //   || ((oldY > y) && (floatInsideY.data <= y)))
      if (((oldY <= y) && (floatInsideY.data > y))
          // 下向きの辺。点Pがy軸方向について、始点と終点の間にある。ただし、始点は含まない。(ルール2)
          || ((oldY > y) && (floatInsideY.data <= y))) {
        // ルール1,ルール2を確認することで、ルール3も確認できている。
        // 辺は点pよりも右側にある。ただし、重ならない。(ルール4)
        // 辺が点pと同じ高さになる位置を特定し、その時のxの値と点pのxの値を比較する。
        float vt = (y - oldY) / (floatInsideY.data - oldY);
        if (x < (oldX + (vt * (floatInsideX.data - oldX)))) {
          ++cn;
        }
      }
      oldX = floatInsideX.data;
      oldY = floatInsideY.data;
    }
    //cnが奇数か偶数か判定する。
    if (cn % 2 == 1)
    {
      //奇数・範囲内・終了
      break;
    }
    else
    {
      //偶数・範囲外・継続
      //何もしない。
    }
    oldX = NAN;
    oldY = NAN;

    memset(tzid, 0, sizeof(tzid));

    uint32_t percentage = umap(fileSize - binFile.available32(), 0, fileSize, 0, 100);
    sevenSegment.setDisplayDigit(percentage / 10, 4);
    sevenSegment.setDisplayDigit(percentage % 10, 5, true);
  }

  binFile.close();

  if (tzid[0] == 0)
  {
    goto ERR;
  }
  else
  {
    currentOffsetMinutes = offset.data;
  }
  isSecondDisp = true;
  return;

ERR:
  //todo　エラーの表示
  /* 設定変更しない。
    set_zone(0);
    tzid[0] = 'U';
    tzid[1] = 'T';
    tzid[2] = 'C';
    tzid[3] = '\0';
  */
  isSecondDisp = true;
  return;
}

void GpsOn()
{
  digitalWrite(GPS_OFF_PIN, LOW);
}

void GpsOff()
{
  digitalWrite(GPS_OFF_PIN, HIGH);
}

//標準関数mapの符号無し版
uint32_t umap(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
