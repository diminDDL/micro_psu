#include <Arduino.h>

uint8_t PPSSymbol[] = {
  B10101,
  B11011,
  B00000,
  B01110,
  B10101,
  B10101,
  B10001,
  B01110
};

uint8_t PPSPart1[] = {
  B00000,
  B00000,
  B11011,
  B10110,
  B11011,
  B10010,
  B00000,
  B00000
};

uint8_t PPSPart2[] = {
  B00000,
  B00000,
  B00011,
  B10100,
  B00010,
  B00001,
  B00110,
  B00000
};

uint8_t PDConnected[] = {
  B00000,
  B01010,
  B01010,
  B11111,
  B10001,
  B10001,
  B01110,
  B00100
};

uint8_t outEn[] = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};

uint8_t outEnAnim[5][8] = {
  {B00000, B00001, B10000, B11111, B10000, B00001, B00000, B00000},
  {B00000, B10000, B01000, B11111, B01000, B10000, B00000, B00000},
  {B00000, B01000, B00100, B11111, B00100, B01000, B00000, B00000},
  {B00000, B00100, B00010, B11111, B00010, B00100, B00000, B00000},
  {B00000, B00010, B00001, B11111, B00001, B00010, B00000, B00000}
};

uint8_t outDis[] = {
  B00001,
  B00001,
  B00001,
  B11111,
  B00001,
  B00001,
  B00001,
  B00000
};

uint8_t error[] = {
  B00000,
  B00000,
  B10001,
  B01010,
  B00100,
  B01010,
  B10001,
  B00000
};

uint8_t apply[] = {
  B00000,
  B00000,
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000
};

