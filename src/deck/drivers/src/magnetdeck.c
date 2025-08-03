#define DEBUG_MODULE "MAGNET_DECK"
#include "debug.h"

#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "platform_defaults.h"

#include "log.h"

void magnetDeckTask();

static void magnetInit() {
  DEBUG_PRINT("Hello Crazyflie 2.1 deck world!\n");

  pinMode(DECK_GPIO_IO1, OUTPUT);

  xTaskCreate(magnetDeckTask, MAGNETDECK_TASK_NAME, MAGNETDECK_TASK_STACKSIZE, NULL, MAGNETDECK_TASK_PRI, NULL);
}

static bool magnetTest() {
  return true;
}

void magnetDeckTask() {
  systemWaitStart();
  TickType_t xLastWakeTime = xTaskGetTickCount();

  logVarId_t logIdControllerCmdThrust = logGetVarId("controller", "cmd_thrust");

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(100)); // 10 Hz

    if (logGetFloat(logIdControllerCmdThrust) > PID_VEL_THRUST_BASE) {
      digitalWrite(DECK_GPIO_IO1, HIGH);
    } else {
      digitalWrite(DECK_GPIO_IO1, LOW);
    }
  }
}

static const DeckDriver magnetDriver = {
  .name = "magnet",

  .usedGpio = DECK_USING_IO_1,

  .init = magnetInit,
  .test = magnetTest,
};

DECK_DRIVER(magnetDriver);