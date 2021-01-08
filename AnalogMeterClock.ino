#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>

#include <time.h>
#include <MsTimer2.h>
#include <TinyGPS++.h>

#include "unions.h"
#include "AnalogMeterClock.h"

TinyGPSPlus gps;

SdFat SD;
char tzid[40] = {'U', 'T', 'C'};

//前回参照時のスイッチの状態
SwitchState previousTimeSwitchState = SwitchState::OFF;
SwitchState previousZoneSwitchState = SwitchState::OFF;
SwitchState previousUpSwitchState = SwitchState::OFF;
SwitchState previousDownSwitchState = SwitchState::OFF;

//現在のスイッチ押下判定状況
//直接の参照は許可しない。取得関数経由で取得すること。
SwitchPressedState TimeSwitchPressed = SwitchPressedState::NotPressed;
SwitchPressedState ZoneSwitchPressed = SwitchPressedState::NotPressed;
SwitchPressedState UpSwitchPressed = SwitchPressedState::NotPressed;
SwitchPressedState DownSwitchPressed = SwitchPressedState::NotPressed;

//秒メーターをプログレスメーターとして使用するかどうか
bool isMeterProgressDisplaying = false;

void setup()
{
  //シリアルポート開始
  Serial.begin(9600);
  while (!Serial)
  {
    //この待機は、Unoでは必要無い。USB機能のついているマイコンでのみ必要。って下に書いてある。
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //gps情報の送信レートを設定
  Serial.println(F("$PMTK314,5,1,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,5,0*29"));

  //タイムゾーンの設定
  set_zone(0);    //UTC
  set_dst(NULL);  //サマータイム設定無し

  //タイマー割り込み設定
  MsTimer2::set(1000, timerFire);
  MsTimer2::start();

  //ピンモード設定
  pinMode(TIME_UPDATE_PIN, INPUT_PULLUP);
  pinMode(ZONE_UPDATE_PIN, INPUT_PULLUP);
  pinMode(OFFSET_UP_PIN, INPUT_PULLUP);
  pinMode(OFFSET_DOWN_PIN, INPUT_PULLUP);
}

void loop()
{
  //Switchの状態を監視、更新
  updateTimeSwitchState();
  
  //システム時間の更新
  setSystemTimeFromGPS();
}

//タイマー割り込みハンドラ
void timerFire()
{
  //システム時間を一秒すすめる。
  system_tick();

  //アナログメーターへ出力
  time_t nowTime = time(NULL);
  tm timeStruct;
  localtime_r( nowTime, &timeStruct);
  //秒メーターがプログレスメーターとして使用されてい無い時
  if (!isMeterProgressDisplaying)
  {
    analogWrite(SEC_PIN, map(timeStruct.tm_sec, 0, MAX_SECOND, 0, MAX_ANALOG_WRITE_VALUE)); 
  }
  analogWrite(MIN_PIN, map(timeStruct.tm_min, 0, MAX_MINIUTE, 0, MAX_ANALOG_WRITE_VALUE));
  analogWrite(HOUR_PIN, map(timeStruct.tm_hour, 0, MAX_HOUR, 0, MAX_ANALOG_WRITE_VALUE)); 
}

//１時間おきにシステム時間を更新します。Serialの読み込みも行っているため、継続して呼び出す必要がある。
void setSystemTimeFromGPS()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    gps.encode(c);
    if (gps.time.isValid() && gps.date.isValid())
    {
      if (gps.time.age() > 1500)
      {
        return; //情報が古い場合は更新しない。age関数はミリ秒を返す。
      }

      //現在時刻の取得
      time_t nowTime = time(NULL);
      tm timeStruct;
      localtime_r( nowTime, &timeStruct);

      if (needsUpdate(&timeStruct))
      {
        struct tm rtc_time;
        rtc_time.tm_sec = gps.time.second();
        rtc_time.tm_min = gps.time.minute();
        rtc_time.tm_hour = gps.time.hour();
        rtc_time.tm_mday = gps.date.day();
        rtc_time.tm_wday = 0;
        rtc_time.tm_mon = gps.date.month() - 1;    //tm構造体は0-11の範囲なので１引く
        rtc_time.tm_year = gps.date.year() - 1900; //tm構造体は1900年起点なので1900を引く
        rtc_time.tm_yday = 0;
        rtc_time.tm_isdst = 0;

        set_system_time(mk_gmtime(&rtc_time));
      }
    }
  }
}

//更新するかどうかを取得します。
bool needsUpdate(const tm *timeStruct)
{
  //毎時、1分0秒のときに更新する
  if (timeStruct->tm_min == 1 && timeStruct->tm_sec == 0)
  {
    return true;
  }
  else
  {
    SwitchPressedState pressedState = getIsTimeSwitchPressed();
    //短押しと長押しのときは、更新する。
    return pressedState == SwitchPressedState::ShortPressed || pressedState == SwitchPressedState::LongPressed;
  }
}

//時間設定スイッチの状態を更新します。
void updateTimeSwitchState()
{
  static unsigned long previousMillis = 0;
  if (previousTimeSwitchState == SwitchState::OFF)
  {
    if (digitalRead(TIME_UPDATE_PIN) == SW_ON)
    {
      //offからonになったとき
      previousTimeSwitchState == SwitchState::ON;
      previousMillis = millis();
    }
    else { /*何もしない*/ }
  }
  else if (previousTimeSwitchState == SwitchState::ON)
  {
    if (digitalRead(TIME_UPDATE_PIN) == SW_OFF)
    {
      //onからoffになったとき
      previousTimeSwitchState == SwitchState::OFF;
      //millisは約50日(49日と17時間ほど)でオーバーフローして0になるが、符号なし整数なのでオーバーフローを考慮する必要は無い。
      //参考　Arduinoで遊ぶページ　millis()のオーバーフロー
      //https://garretlab.web.fc2.com/arduino/lab/millis/
      //押されていた時間ミリ秒
      unsigned long interval = millis() - previousMillis;
      if (interval < CHATTERING_TIME_MS)
      {
        //20ミリ秒未満はチャタリングとみなして無視
        TimeSwitchPressed = SwitchPressedState::NotPressed;
      }
      else if (interval < LONG_PUSH_TIME_MS)
      {
        //短押し
        TimeSwitchPressed = SwitchPressedState::ShortPressed;
      }
      else
      {
        //長押し
        TimeSwitchPressed = SwitchPressedState::LongPressed;
      }
    }
    else { /*何もしない*/ }
  }
  else { /*何もしない*/ }
}

//gpsから現在地を取得し、タイムゾーンを検索してオフセットを設定します。
void setTimeZoneOffset()
{
  //todo　gpsから現在地取得
  float x = 0;
  float y = 0;

  if (gps.location.isValid())
  {
    x = gps.location.lat();
    y = gps.location.lng();
  }
  else
  {
    goto ERR;
  }

  //ファイル開く
  File binFile = SD.open(F("DATA.bin"));
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
        Serial.println(tzid);
        Serial.flush();
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
          || ((oldY > y) && (floatInsideY.data <= y)))
      {
        // ルール1,ルール2を確認することで、ルール3も確認できている。
        // 辺は点pよりも右側にある。ただし、重ならない。(ルール4)
        // 辺が点pと同じ高さになる位置を特定し、その時のxの値と点pのxの値を比較する。
        float vt = (y - oldY) / (y - oldY);
        if (x < (oldX + (vt * (floatInsideX.data - oldX))))
        {
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
  }

  binFile.close();

  if (tzid[0] == 0)
  {
    goto ERR;
  }
  else
  {
    //オフセットは分で取得できる。
    //set_zone関数の引数は秒
    set_zone(offset.data * 60);
  }
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
  return;
}

//時間設定ボタンの押下判定を取得します。
//時間設定ボタンの押下判定を初期化(NotPressedに設定)します。
SwitchPressedState getIsTimeSwitchPressed()
{
  SwitchPressedState state = TimeSwitchPressed;
  TimeSwitchPressed = SwitchPressedState::NotPressed;
  return state;
}