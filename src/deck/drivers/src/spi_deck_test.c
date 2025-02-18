#include "spi_test_deck.h"

void spiTestDeckInit(DeckInfo *info)
{
  // Initialize the deck here
}

bool spiTestDeckTest(void)
{
  // Test the deck here
  return true;
}

static const DeckDriver bicopter_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "spiTestDeck",
  .usedPeriph = DECK_USING_SPI,
  //.usedGpio =,
  .init = spiTestDeckInit,
  .test = spiTestDeckTest,
};

DECK_DRIVER(bicopter_deck);