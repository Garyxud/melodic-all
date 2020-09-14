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

char areyou[] = "Hello, are you RX?"; //18
unsigned char * MaxAreYouPos; //areyou + PREAMBLE_SIZE

char imfrom[] = "Yes, I'm RX"; //11

unsigned char *serialPreamblePos;
unsigned char *serialAreYouPos;

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
}; // Define a static TX address

//unsigned char rx_buf[TX_PLOAD_WIDTH];
//unsigned char tx_buf[TX_PLOAD_WIDTH];
//***************************************************

//unsigned char g_status;

char caracterActual = -1;

void mycopy(void * dst, void * orig, unsigned int tam)
{
  uint8_t * dptr, *optr;
  dptr = (uint8_t*) dst;
  optr = (uint8_t*) orig;
  
  unsigned int count = 0;
  for(count = 0; count<tam;count++)
  {
     *dptr = *optr;
     dptr++;
     optr++;
  }
}
class Radio
{
  public:
    Radio()
    {
       _available = 0;
    }
    ~Radio(){};

    int Available()
    {
       if (_available)
         return _available;
       
       return _readyForRead();
    }

    
    void ReadBytes(void * dst, unsigned int req) //Blocking call
    {
       if(req <= _available)
       {
         mycopy(dst, _buff, req);
         _available = _available - req;
         mycopy(_buff, _buff+req, _available);
         return;
       }
       else
       {
         uint8_t * ptr = (uint8_t *) dst;
         mycopy(ptr, _buff, _available);
         ptr += _available;
         unsigned int left = req - _available;
         
         _available = 0;
         while(left > 0)
         {
             while(!_readyForRead());
             uint8_t dataLength;
             dataLength = SPI_Read(0x60);
             SPI_Read_Buf(RD_RX_PLOAD, _buff, dataLength);
             if(dataLength > TX_PLOAD_WIDTH) continue; //Esto es por si acaso...
             if(dataLength <= left)
             {
               mycopy(ptr, _buff, dataLength);
               ptr += dataLength;
               left -= dataLength;
             }
             else
             {
               mycopy(ptr, _buff, left);
               _available = dataLength - left;
               mycopy(_buff, _buff+left, _available);
               left = 0;
             }
             /*
             Despues de leer, siempre resetear STATUS register
             Si no se hace, no funciona correctamente... Consultar
             Datasheet 
             */
             _status = SPI_Read(STATUS);
             SPI_RW_Reg(WRITE_REG+STATUS,_status);
         }
       }
        
    }
    private:
       uint8_t _buff[TX_PLOAD_WIDTH+1];
       uint8_t _available = 0;
       uint8_t _status;
       uint8_t _fifostatus;
       
       int _readyForRead()
       {
             _status = SPI_Read(STATUS);
             _fifostatus = SPI_Read(FIFO_STATUS); 
             //En teoria, con comprobar RX_EMPTY es suficiente...
             boolean result =  ((_status&RX_DR) || !(_fifostatus&RX_EMPTY)) && (_fifostatus&TX_EMPTY);
             
             SPI_RW_Reg(WRITE_REG+STATUS,_status); //RESETEAMOS LOS BITS RX_DR, (y TX_DR y MAX_RT, aunque no los tenemos en cuenta)
             return result;
       }
        
    
};
Radio radio;
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
    if (stream->available() > 0)
      buff[bytes++] = stream->read();
  }
}


uint16_t radioFrameReceived(char* buffer, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
  if (*preamblePos == maxPrePos)
  {
    //Serial.println("Preambulo recibido");
    *preamblePos = pre;
    radio.ReadBytes(buffer, INFO_SIZE);
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
      
      radio.ReadBytes(buffer+INFO_SIZE, dsize + FCS_SIZE);
      return dsize;
    }
    //Serial.println("ERROR");
    return 0;
  }

  char car;
  
  radio.ReadBytes(&car,1); 
  //Serial.print("leido: ");Serial.println(car);  
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
  //  attachInterrupt(1, _ISR, LOW); // interrupt enable
  Serial.begin(115200);
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");
  Serial.println(status,HEX);      // There is read the mode’s status register, the default value should be ‘E’  
  Serial.println("*****************RX_Mode start******************************R");
  RX_Mode();                        // set RX mode
}

int paylsize;

void loop() 
{
  
  if (radio.Available())
  {
    if (paylsize=radioFrameReceived(buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
    {
        Serial.write(preamble, PREAMBLE_SIZE);   
        Serial.write(buf, INFO_SIZE + paylsize + FCS_SIZE);
        Serial.flush();
    }
   
  }
  else if (Serial.available() > 0)
  {
    caracterActual = Serial.peek();

    
    if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
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
      digitalWrite(MOSI, 1);    // output 'unsigned char', MSB to MOSI
    }
    else
    {
      digitalWrite(MOSI, 0);
    }
    digitalWrite(SCK, 1);                      // Set SCK high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISO) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCK, 0);         	// ..then set SCK low again
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

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                  // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: RX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * RX Mode, set RX address, writes RX payload width,
 * select RF channel, datarate & LNA HCURR.
 * After init, CE is toggled high, which means that
 * this device is now ready to receive a datapacket.
/**************************************************/
void RX_Mode(void)
{
  digitalWrite(CE, 0);
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  //SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      // Disable Auto.Ack:PipeX
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  //SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27); 
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..
  
  SPI_RW_Reg(WRITE_REG + 0x1D, 0x04);  //Activar DPL (EN_DPL bit) DPL = Dynamic Payload Length (en registro FEATURE)
  SPI_RW_Reg(WRITE_REG + 0x1C, 0x01);    //Activar DPL en Pipe0 (DPL_PO) (en registro DYNPD)
  
  //SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0A);
  digitalWrite(CE, 1);                             // Set CE pin high to enable RX device
  //  This device is now ready to receive one packet of 16 unsigned chars payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}
/**************************************************/


   
  /*** TEST 2 ***/
 /* 
  unsigned int tam = 32;
  unsigned char auxbuf[tam];
  int reqs = tam;//TX_PLOAD_WIDTH;
  unsigned int  dataLength;
  for(;;)
  {
    if(radio.Available())
    {
  
      radio.ReadBytes(auxbuf,reqs);
     //delay(1000);
     // Serial.println("###########");
      for(int i=0; i<reqs; i++)
      {
          Serial.print(" ");
        //Serial.print((char)auxbuf[i]);                              // print rx_buf
          Serial.print(auxbuf[i], HEX);                              // print rx_buf

      }
      
      Serial.println();
    }
    else if (Serial.available() > 0)
    {
      caracterActual = Serial.peek();
  
      
      if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
        Serial.write(imfrom, IMFROM_SIZE);
  
      Serial.read();
      
    }
  }
*/
