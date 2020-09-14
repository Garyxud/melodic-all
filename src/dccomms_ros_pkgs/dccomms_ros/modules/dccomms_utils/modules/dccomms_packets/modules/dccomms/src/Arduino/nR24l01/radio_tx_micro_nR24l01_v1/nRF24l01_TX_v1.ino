#include <SPI.h>

/*********************************************************************
**  Device:  nRF24L01+                                              **
**  File:   EF_nRF24L01_TX.c                                        **
**                                                                  **
**                                                                  **
**  Copyright (C) 2011 ElecFraks.                                   **
**  This example code is in the public domain.                      **
**                                                                  **
**  Description:                                                    **
**  This file is a sample code for your reference.                  **
**  It's the v1.1 nRF24L01+ by arduino                              **
**  Created by ElecFreaks. Robi.W,24 July 2011                      **
**                                                                  **
**  http://www.elecfreaks.com                                       **
**                                                                  **
**   SPI-compatible                                                 **
**   CS - to digital pin 8                                          **
**   CSN - to digital pin 9  (SS pin)                               **
**   SCK - to digital pin 10 (SCK pin)                              **
**   MOSI - to digital pin 11 (MOSI pin)                            **
**   MISO - to digital pin 12 (MISO pin)                            **
**   IRQ - to digital pin 13 (MISO pin)                             **
*********************************************************************/


#include "NRF24L01.h"

//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 unsigned chars TX payload
#define TX_UNIT 32

#define FCS_SIZE 4 
#define INFO_SIZE 4
#define PREAMBLE_SIZE 10

#define AREYOU_SIZE 18
#define IMFROM_SIZE 11

#define BUFFER_SIZE 1500

char buf[BUFFER_SIZE];

bool BigEndian;

char preamble[] = {0x55,0x55,0x55,0x55,0x55,0x55,
0x55,0x55,0x55,0x55};

unsigned char * MaxPrePos; //preamble + PREAMBLE_SIZE

char areyou[] = "Hello, are you TX?"; //18
unsigned char * MaxAreYouPos; //areyou + PREAMBLE_SIZE

char imfrom[] = "Yes, I'm TX"; //11

unsigned char *serialPreamblePos;
unsigned char *serialAreYouPos;

char caracterActual = -1;
int paylsize;

bool IsBigEndian()
{
	uint32_t word = 0x1;
	uint8_t * byte = (uint8_t *)&word;
	return *byte != 0x1;
}

void readNBytes(char* buff, int tam, Stream * stream)
{
	int bytes = 0;
	while (bytes < tam)
	{
		if (stream->available()>0)
			buff[bytes++] = stream->read();
	}
}

uint16_t radioFrameReceived(Stream * s, char* buffer, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
  if (*preamblePos == maxPrePos)
  {
    *preamblePos = pre;
    readNBytes(buffer, INFO_SIZE , s);
    uint16_t dsize;
    uint16_t *fdsize = (uint16_t *)(buffer+2);
    if(BigEndian)
    {
      dsize = *fdsize;
    }
    else
    {
      dsize = ((*fdsize) << 8) | ((*fdsize) >> 8);
    }
    
    if(dsize <= BUFFER_SIZE-FCS_SIZE)
    {
      readNBytes(buffer+INFO_SIZE, dsize + FCS_SIZE, s);
      return dsize;
    }
    
    return 0;
  }

  char car = caracterActual;
  if (car == **preamblePos)
  {
    (*preamblePos)++;
  }
  else
  {
    *preamblePos = pre;
  }

  return 0;
}

boolean commandReceived(Stream * s, char* buffer, int buffSize, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
	if (*preamblePos == maxPrePos)
	{
		readNBytes(buffer, buffSize, s);
		*preamblePos = pre;
		return true;
	}

	char car = caracterActual;
		

	if (car == **preamblePos)
	{
		(*preamblePos)++;
	}
	else
	{
		*preamblePos = pre;
	}


	return false;
}


unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
}; // Define a static TX address

unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
//***************************************************

void clearStatus()
{
  unsigned char status;
  status = SPI_Read(STATUS);
  SPI_RW_Reg(WRITE_REG+STATUS,status);// clear RX_DR or TX_DS or MAX_RT interrupt flag
}

void sendRfPayload(unsigned char ** ptr, uint8_t nb)
{
    
    boolean enviado = false;
    while(!enviado)
    {
        clearStatus();
        SPI_Write_Buf(WR_TX_PLOAD, *ptr,nb);       // write playload to TX_FIFO
        unsigned char status;
        do
        {
          status = SPI_Read(STATUS);
        } while(!(status&TX_DS) && !(status&MAX_RT));
        
        if(!(status&MAX_RT))
        {
           enviado = true;
        }
        
    }

    *ptr += nb;
}

void radioWrite(void * _buf, uint32_t tam)
{
  unsigned int units = tam / TX_UNIT;
  unsigned int left = tam % TX_UNIT;
  uint8_t * ptr = (uint8_t *)_buf;
  uint8_t * maxptr = ptr + (TX_UNIT)*units;
  while(ptr != maxptr)
  {
    sendRfPayload(&ptr, TX_UNIT);
  }
  if(left>0)
  {
    sendRfPayload(&ptr, left);
  }
}

void setup() 
{
  BigEndian = IsBigEndian();
	
  MaxPrePos = (unsigned char *) preamble + PREAMBLE_SIZE;
  MaxAreYouPos = (unsigned char*) areyou + AREYOU_SIZE;
    
  serialPreamblePos = (unsigned char*)preamble;
  serialAreYouPos = (unsigned char*)areyou;
  
  pinMode(CE,  OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(MOSI,  OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(IRQ, INPUT);
  //  attachInterrupt(1, _ISR, LOW);// interrupt enable
  Serial.begin(115200);
  while (!Serial) {
; // wait for serial port to connect. Needed for Leonardo only
}
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");    
  Serial.println(status,HEX);     // There is read the mode’s status register, the default value should be ‘E’
  Serial.println("*******************TX_Mode Start****************************");
  TX_Mode();                       // set TX mode
}

void loop() 
{

  if (Serial.available()>0)
	{
      	    caracterActual = Serial.peek();

	    if (paylsize=radioFrameReceived(&Serial, buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
	    {
		 radioWrite(preamble, PREAMBLE_SIZE);
		 radioWrite(buf, INFO_SIZE + paylsize + FCS_SIZE);
            }

	    else if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
		Serial.write(imfrom, IMFROM_SIZE);

    	    Serial.read();
	}
}

//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);			// chip enable
  digitalWrite(CSN, 1);                 // Spi disable	
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      digitalWrite(MOSI, 1);
    }
    else
    {
      digitalWrite(MOSI, 0);
    }
    digitalWrite(SCK, 1);
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISO) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCK, 0);
  }
  return(Byte);           	        // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSN, 0);                   // CSN low, init SPI transaction
  status = SPI_RW(reg);                   // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSN, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSN, 0);           // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  digitalWrite(CSN, 1);          // CSN high, terminate SPI communication
  
  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSN, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSN, 1);                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSN, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                   // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: TX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void)
{
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
 // SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      // Disable Auto.Ack:PipeX
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
  //SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27);
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
  
  SPI_RW_Reg(WRITE_REG + 0x1D, 0x04);  //Activar DPL (EN_DPL bit) DPL = Dynamic Payload Length (en registro FEATURE)
  SPI_RW_Reg(WRITE_REG + 0x1C, 0x01);    //Activar DPL en Pipe0 (DPL_PO) (en registro DYNPD)
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

  //SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0A);

  digitalWrite(CE, 1);
}



