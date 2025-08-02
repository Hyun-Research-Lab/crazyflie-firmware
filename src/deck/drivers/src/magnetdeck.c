#define DEBUG_MODULE "MAGNET_DECK"
#include "debug.h"

#include "deck.h"

#include "FreeRTOS.h"
#include "task.h"

static void magnetInit() {
  DEBUG_PRINT("Hello Crazyflie 2.1 deck world!\n");

  pinMode(DECK_GPIO_IO1, OUTPUT);
}

static bool magnetTest() {
  digitalWrite(DECK_GPIO_IO1, HIGH);
  vTaskDelay(M2T(1000));
  digitalWrite(DECK_GPIO_IO1, LOW);

  DEBUG_PRINT("Hello test passed!\n");

  return true;
}

static const DeckDriver magnetDriver = {
  .name = "magnet",

  .usedGpio = DECK_USING_IO_1,

  .init = magnetInit,
  .test = magnetTest,
};

DECK_DRIVER(magnetDriver);