#include <time.h>
#include <MsTimer2.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

time_t oldTime = 0;

#define SEC_PIN 5
#define MIN_PIN 6
#define HOUR_PIN 9

//タイマー割り込みハンドラ
void timerFire() {
  //システム時間を一秒すすめる。
  system_tick();

  //アナログメーターへ出力
  time_t timenow = time(NULL);
  tm timeStruct;
  localtime_r(&timenow, &timeStruct);

  analogWrite(SEC_PIN, map(timeStruct.tm_sec, 0, 60, 0, 255));
  analogWrite(MIN_PIN, map(timeStruct.tm_min, 0, 60, 0, 255));
  analogWrite(HOUR_PIN, map(timeStruct.tm_hour, 0, 24, 0, 255));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  set_zone(+9 * ONE_HOUR);
  set_dst(NULL);

  //タイマー割り込み設定
  MsTimer2::set(1000, timerFire);
  MsTimer2::start();
}

void loop() {
  //現在時刻の取得

  //システム時間の更新
  setSystemTimeFromGPS();
}

//１時間おきにシステム時間を更新します。
void setSystemTimeFromGPS(){
  while (Serial.available() > 0){
    char c = Serial.read();
    gps.encode(c);
    if(gps.time.isUpdated()){
      if(gps.date.year() < 2000) return; //正常に日付が取得できて無い場合は終了。

      //現在時刻の取得
      time_t timenow = time(NULL);
      tm timeStruct;
      localtime_r(&timenow, &timeStruct);

      //毎時、0分0秒のときに更新する
      if(timeStruct.tm_min == 0 && timeStruct.tm_sec == 0){
        struct tm rtc_time;
        rtc_time.tm_sec = gps.time.second();
        rtc_time.tm_min = gps.time.minute(); 
        rtc_time.tm_hour = gps.time.hour(); 
        rtc_time.tm_mday = gps.date.day(); 
        rtc_time.tm_wday = 0; 
        rtc_time.tm_mon = gps.date.month() - 1; //tm構造体は0-11の範囲なので１引く
        rtc_time.tm_year = gps.date.year() - 1900; //tm構造体は1900年起点なので1900を引く
        rtc_time.tm_yday = 0; 
        rtc_time.tm_isdst = 0;
  
        set_system_time( mk_gmtime(&rtc_time) );
      }
    }
  }
}
