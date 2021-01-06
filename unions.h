//フロート型の中身を操作できる共用体
typedef union
{
  float data;
  char byteData[4];
} FLOAT_INSIDE;

typedef union
{
  int32_t data;
  char byteData[4];
} INT_INSIDE;

typedef union
{
  int16_t data;
  char byteData[2];
} SHORT_INSIDE;
