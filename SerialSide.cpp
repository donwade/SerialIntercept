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

// do not enable when driven by the S3K app 
//#define ECHO_TELNET_TO_SERIAL


#if 1
// danger uart 1 is the flash (ouch).
HardwareSerial HostSide(0);
#else
#define HostSide Serial
#error caution serial echos back to term :(
#endif

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

void send2TelenetMonitor(char *what)
{
	//always
    telnet.println(what);


    // normally not enabled as this chatter
    // could be sent into the windows app and mess it up.
    //
    // if your not driving it from the S31110 app 
    // turn it on so you can watch.
    
    #ifdef ECHO_TELNET_TO_SERIAL
	HostSide.printf("%s", what);
	#endif 
}

//--------------------------------------------------------------

#define WIDE 32
void dump(char * tag, uint8_t *array, int size)
{
	int forward = 0;
	
	sprintf(bigBuffer, "T=%9d %s[%d] : ", millis(), tag, size);
	send2TelenetMonitor(bigBuffer);
	
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

		send2TelenetMonitor(bigBuffer);
		
		size -= run;
	}

}


//--------------------------------------------------------------
void process_fromS3(void)
{
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

	  HostSide.print(achar);
	}
}

//-------------------------------------------------------------
// allow 2 different sources to send to the S3
// one is from the serial stream from the PC
// the other is from an internal test stub in this program

void toS3(char achar)
{
	digitalWrite(PIN_LED, 1);

#ifdef LOOK4SYNC
	  if (!bSyncFoundLeft)
	  {
		  if (achar == SYNC_START[syncStartLeft])
		  {
			  if (syncStartLeft == 3)
			  {
				  Left[indexLeft++] = achar;
				  bSyncFoundLeft = true;
				  syncEndLeft = 0;
			  }
			  else
			  {
				  digitalWrite(PIN_LED, 1);
				  syncStartLeft++;
				  Left[indexLeft++] = achar;
			  }
		  }
		  else
		  {
			  //sync still not found. or sync miss
			  syncStartLeft = 0;
			  indexLeft = 0;
			  bSyncFoundLeft = false;
			  digitalWrite(PIN_LED, 0);
		  }
	  }
	  else
#endif

	  {
		// sync has been found... collect payload
			Left[indexLeft++] = achar;

#ifdef LOOK4SYNC
			if (achar == SYNC_END[syncEndLeft])
			{
				if (syncEndLeft == 3)
				{
					syncEndLeft = 0;

					
					dump("TO SK3:", Left, MIN(MAX_CHARS, indexLeft));
					indexLeft = 0;
					syncStartLeft = 0;
					bSyncFoundLeft = false;
				}
				else
				{
					digitalWrite(PIN_LED, 1);
					syncEndLeft++;
				}
			}
#endif
		}
	sk3k1110.write(achar);
}

//-------------------------------------------------------------
void inject_toS3(char achar)
{
	toS3(achar);
}
//--------------------------------------------------------------
void process_toS3(void)
{
	if (HostSide.available()) 
	{
		uint8_t achar;
		achar = HostSide.read();
		toS3(achar);
	}
}

//-------------------------------------------------------------

void request_firmware(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START); i++) 
	{
		toS3(SYNC_START[i]);
	}

	for (i = 0; i < ELEMENTS(GET_FIRMWARE); i++) 
	{
		toS3(GET_FIRMWARE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END); i++) 
	{
		toS3(SYNC_END[i]);
	}
}

//----------------------------------------------------------
static int testRequestCnt =0;
void rs232_loop() 
{

  if (testRequestCnt++ > 10000)
  {
  	request_firmware();
  	testRequestCnt=0;
  }

  process_toS3();
  process_fromS3();
}

