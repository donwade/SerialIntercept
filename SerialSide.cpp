/*
  SerialPassthrough sketch
*/

#include "ESPTelnet.h"
#define PIN_LED 2

//-------------------------------------------
#define MIN(a,b)  (a < b) ? a : b

// give hardware serial a heads up we are not standard                                     
#define RX2 18
#define TX2 19

extern ESPTelnet telnet; 

#include <HardwareSerial.h>
HardwareSerial sk3k1110(2);

uint8_t SYNC_START[] = {0XFD, 0xFC, 0xFB, 0xFA};
uint8_t SYNC_END  [] = {4,3,2,1};

uint8_t GET_FIRMWARE [] = { 02, 00, 00, 00 };

#define ELEMENTS(x) (sizeof(x)/sizeof(x[0]))

#define LOOK4SYNC   // reverse eng of S3KM1110


#if 1
// danger uart 1 is the flash (ouch).
HardwareSerial HostSide(0);
#else
#define HostSide Serial
#error caution serial echos back to term :(
#endif

void request_firmware(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START); i++) 
	{
		sk3k1110.write(SYNC_START[i]);
	}

	for (i = 0; i < ELEMENTS(GET_FIRMWARE); i++) 
	{
		sk3k1110.write(GET_FIRMWARE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END); i++) 
	{
		sk3k1110.write(SYNC_END[i]);
	}
}

uint32_t lastReading = 0;

void rs232_setup(void)
{
    HostSide.begin(115200);

	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, 1);
	
    sk3k1110.setPins(RX2, TX2);  // runtime overide
    sk3k1110.begin(115200);

    HostSide.printf("used for interept of serial stream for S3KM11110\n");

	delay(3000);
	digitalWrite(PIN_LED, 0);
	
    while (HostSide.available()) HostSide.read();
    while (sk3k1110.available()) sk3k1110.read();

    HostSide.printf("looping\n");
    request_firmware();
    
    
}

#define MAX_CHARS 80

uint8_t Left[MAX_CHARS];
uint8_t Right[MAX_CHARS];

int indexLeft = 0;
int indexRight = 0;

int syncStartLeft = 0;
int syncStartRight = 0;

int syncEndLeft = 0;
int syncEndRight = 0;


bool bSyncFoundLeft = false;
bool bSyncFoundRight = false;


char bigBuffer[2000];

//--------------------------------------------------------------

void sendit(char *what)
{
    telnet.println(what);
	HostSide.printf("%s", what);
}

//--------------------------------------------------------------

#define WIDE 32
void dump(char * tag, uint8_t *array, int size)
{

	int forward = 0;
	
	sprintf(bigBuffer, "%s[%d] : ", tag, size);
	sendit(bigBuffer);
	
	while (size > 0)
	{
		bigBuffer[0] = 0;
		
		int run = MIN(size, WIDE);
		int len = 0;
		int hold;

		hold = forward;
		for( int i = 0; i < run; i++)
		{
			if (i < run)
				len += sprintf(&bigBuffer[len], "%02X ", array[forward]);
			else
				len += sprintf(&bigBuffer[len], "   ", array[forward]);
			forward++;
		}
		
		len += sprintf(&bigBuffer[len], "===");

		forward = hold;
		for( int i = 0; i < run; i++)
		{
            
			len += sprintf(&bigBuffer[len], "%c", array[forward] < 0x20 ? '.': array[forward] > 0x7F? '_' : array[forward]);
			forward++;			
		}
		
		len += sprintf(&bigBuffer[len], "\n");

		sendit(bigBuffer);
		
		size -= run;
	}

}

//--------------------------------------------------------------

static int testRequestCnt =0;

void rs232_loop() 
{

  if (testRequestCnt++ > 10000)
  {
  	request_firmware();
  	testRequestCnt=0;
  }
  
  if (HostSide.available())
  {
    uint8_t achar;
    achar = HostSide.read(); 

	if (!bSyncFoundLeft && achar == SYNC_START[syncStartLeft])
	{
		if (syncStartLeft == 3)
		{
			Left[indexLeft++] = achar;
			bSyncFoundLeft = true;
		}
		else
		{
			syncStartLeft++;
			Left[indexLeft++] = achar;
			goto doRight;
		}
	}
	else
	{
		syncStartLeft = 0;
		indexLeft = 0;
		bSyncFoundLeft = false;
		goto doRight;
	}
		

	Left[indexLeft++] = achar;
	if (indexLeft == MAX_CHARS || achar == 0xA)
	{
		dump("TO SK3:", Left, MIN(MAX_CHARS, indexLeft -1));
		indexLeft = 0;
		syncStartLeft = 0;
		bSyncFoundLeft = false;
	}

    sk3k1110.write(achar);  // read it and send it out HostSide1 (pins 0 & 1)
  }

doRight:


  if (sk3k1110.available()) 
  {
	  uint8_t achar;
	  achar = sk3k1110.read();
	  digitalWrite(PIN_LED, 1);

	  //HostSide.print(achar, HEX); HostSide.print("  ");

#ifdef LOOK4SYNC
	  if (!bSyncFoundRight)
	  {
	  	  if (achar == SYNC_START[syncStartRight])
	  	  {
			  HostSide.print(syncStartRight+1); 
			  if (syncStartRight == 3)
			  {
				  Right[indexRight++] = achar;
				  bSyncFoundRight = true;
				  syncEndRight = 0;
			  }
			  else
			  {
				  digitalWrite(PIN_LED, 1);
				  syncStartRight++;
				  Right[indexRight++] = achar;
			  }
		  }
		  else
		  {
		  	  //sync still not found. or sync miss
			  syncStartRight = 0;
			  indexRight = 0;
			  bSyncFoundRight = false;
			  digitalWrite(PIN_LED, 0);
	      }
	  }
	  else
#endif

	  {
	  	// sync has been found... collect payload
			Right[indexRight++] = achar;

#ifdef LOOK4SYNC
			if (achar == SYNC_END[syncEndRight])
			{
				if (syncEndRight == 3)
				{
					syncEndRight = 0;

					
					dump("FROM SK3:", Right, MIN(MAX_CHARS, indexRight));
					indexRight = 0;
					syncStartRight = 0;
					bSyncFoundRight = false;
				}
				else
				{
					digitalWrite(PIN_LED, 1);
					syncEndRight++;
				}
			}
#endif
	  }

    HostSide.write(achar);  // read it and send it out HostSide (USB)
  
  	}
}


