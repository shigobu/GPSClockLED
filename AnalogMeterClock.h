//定数
#define SEC_PIN 5
#define MIN_PIN 6
#define HOUR_PIN 9

#define TIME_UPDATE_PIN 3
#define ZONE_UPDATE_PIN 2

#define OFFSET_UP_PIN 4
#define OFFSET_DOWN_PIN 7

#define MAX_SECOND 60
#define MAX_MINIUTE 60
#define MAX_HOUR 24

#define MAX_ANALOG_WRITE_VALUE 255

//スイッチの状態と入力端子の状態の対応
#define SW_ON LOW
#define SW_OFF HIGH

//長押し判定の時間ミリ秒
#define LONG_PUSH_TIME_MS 5000

#define CHATTERING_TIME_MS 20

enum SwitchState
{
  ON,
  OFF
};

enum SwitchPressedState
{
  NotPressed,
  ShortPressed,
  LongPressed
};

enum Switchs 
{
  TimeSwitch,
  ZoneSwitch,
  UpSwitch,
  DownSwitch
};

//関数のプロトタイプ宣言
void timerFire();
void setSystemTimeFromGPS();
void setTimeZoneOffset();
void updateSwitchState(Switchs sw);
SwitchPressedState getIsTimeSwitchPressed();
