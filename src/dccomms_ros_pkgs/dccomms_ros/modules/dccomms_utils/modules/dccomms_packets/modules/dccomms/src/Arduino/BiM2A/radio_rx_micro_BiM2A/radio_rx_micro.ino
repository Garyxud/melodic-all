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

char areyou[] = "Hello, are you RX?"; //18
unsigned char * MaxAreYouPos; //areyou + PREAMBLE_SIZE

char imfrom[] = "Yes, I'm RX"; //11

unsigned char *serialPreamblePos;
unsigned char *serialAreYouPos;
const int analogOutPin = A0;

HardwareSerial * radioStream = &Serial1;

void modoRx()
{
    digitalWrite(TXS, HIGH); //RX enabled
    digitalWrite(RXS, LOW);
}

void radioInit()
{
    pinMode(TXS, OUTPUT);
    pinMode(RXS, OUTPUT);
    modoRx();
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
    if (stream->available() > 0)
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
void setup() {
  Serial.begin(115200);
  //radioStream->begin(19200);
  radioStream->begin(9600);
  //radioStream->begin(57600);

  BigEndian = IsBigEndian();
  
  MaxPrePos = (unsigned char *) preamble + PREAMBLE_SIZE;
  MaxAreYouPos = (unsigned char*) areyou + AREYOU_SIZE;

  serialPreamblePos = (unsigned char*)preamble;
  serialAreYouPos = (unsigned char*)areyou;
  
  radioInit();
}


int paylsize;

void loop()
{
  if (radioStream->available() > 0)
  {
    
    caracterActual = radioStream->peek();
    if (paylsize=radioFrameReceived(radioStream, buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
    {
        Serial.write(preamble, PREAMBLE_SIZE);   
        Serial.write(buf, INFO_SIZE + paylsize + FCS_SIZE);
        Serial.flush();
    }
    radioStream->read();
  }
  else if (Serial.available() > 0)
  {
    caracterActual = Serial.peek();

    
    if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
      Serial.write(imfrom, IMFROM_SIZE);

    Serial.read();
  }
}



