#ifndef NRF24L01_h
#define NRF24L01_h

#include "API.H"

//---------------------------------------------
#define TX_ADR_WIDTH    5   
// 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  1  
// 20 unsigned chars TX payload
//---------------------------------------------
#define CE       8
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode
#define CSN      9
// CSN BIT:  Digital Input     SPI Chip Select
#define SCK      10
// SCK BIT:  Digital Input     SPI Clock
#define MOSI     11
// MOSI BIT: Digital Input     SPI Slave Data Input
#define MISO     12
// MISO BIT: Digital Output    SPI Slave Data Output, with tri-state option
#define IRQ      13
// IRQ BIT:  Digital Output    Maskable interrupt pin
//*********************************************
#endif
