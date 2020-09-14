#define FCS_SIZE 4 
#define INFO_SIZE 4
#define PREAMBLE_SIZE 10

#define AREYOU_SIZE 18
#define IMFROM_SIZE 11

#define BUFFER_SIZE 1500

#define TXS 6
#define RXS 7

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
const int analogOutPin = A0;

HardwareSerial *radioStream = &Serial1;

void modoTx()
{
    digitalWrite(TXS, LOW); //TX enabled
    digitalWrite(RXS, HIGH);
}

void radioInit()
{
    pinMode(TXS, OUTPUT);
    pinMode(RXS, OUTPUT);
    modoTx();
}


char caracterActual = -1;

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
		//Serial.println("Obteniendo payload...");
		if (stream->available()>0)
			buff[bytes++] = stream->read();
	}
}

uint16_t radioFrameReceived(Stream * s, char* buffer, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
  if (*preamblePos == maxPrePos)
  {
    //analogWrite(analogOutPin, 255);
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
      //analogWrite(analogOutPin, 0);
      return dsize;
    }
    //analogWrite(analogOutPin, 0);
    //Serial.println(dsize, HEX);
    //Serial.println(dsize + FCS_SIZE, HEX);
    
    return 0;
  }

  char car = caracterActual;
  if (car == **preamblePos)
  {
    //s->print("Coincide! "); s->println(iteracion);
    (*preamblePos)++;
  }
  else
  {
    //s->print("No coincide..."); s->println(iteracion);
    *preamblePos = pre;
  }

  return 0;
}

boolean commandReceived(Stream * s, char* buffer, int buffSize, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
	if (*preamblePos == maxPrePos)
	{
		//s->println("Comando reconocido!");
		//analogWrite(analogOutPin, 255);
		readNBytes(buffer, buffSize, s);
		*preamblePos = pre;
		//analogWrite(analogOutPin, 0);
		return true;
	}

	char car = caracterActual;
		

	if (car == **preamblePos)
	{
		//s->print("Coincide! "); s->println(iteracion);
		(*preamblePos)++;
	}
	else
	{
		//s->print("No coincide..."); s->println(iteracion);
		*preamblePos = pre;
	}


	return false;
}

char sensorsInfo[40];


void setup() {
	Serial.begin(115200);
	radioStream->begin(9600);

	for (int i = 0; i < 40; i++)
		sensorsInfo[i] = 'i';

        BigEndian = IsBigEndian();
	
	MaxPrePos = (unsigned char *) preamble + PREAMBLE_SIZE;
	MaxAreYouPos = (unsigned char*) areyou + AREYOU_SIZE;

	serialPreamblePos = (unsigned char*)preamble;
	serialAreYouPos = (unsigned char*)areyou;
	
        radioInit();
}

int paylsize;
unsigned int t0,t1;
void loop()
{
  
	if (Serial.available()>0)
	{
		caracterActual = Serial.peek();

		if (paylsize=radioFrameReceived(&Serial, buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
		{/*
                        t0 = millis();
                        t1 = t0;
                        while (t1-t0 < 10){ 
                          t1 = millis();
                          radioStream->write(0x55);
                        };
                        radioStream->write(0x0);
                        radioStream->write(0xff);
                        radioStream->write(0x01);  */  
                    
			radioStream->write(preamble, PREAMBLE_SIZE);
			radioStream->write(buf, INFO_SIZE + paylsize + FCS_SIZE);
                        radioStream->flush();
		}

		else if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
			Serial.write(imfrom, IMFROM_SIZE);

		Serial.read();
	}

}

