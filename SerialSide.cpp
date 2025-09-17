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

uint8_t SYNC_START1[] = {0XFD, 0xFC, 0xFB, 0xFA};
uint8_t SYNC_END1  [] = {4,3,2,1};

uint8_t SYNC_START2[] = {0XAA, 0xBF, 0x10, 0x14};
uint8_t SYNC_END2  [] = {0xFD, 0xFC, 0xFB, 0xFA};



uint8_t GET_FIRMWARE [] = { 02, 00, 00, 00 };
uint8_t SET_REPORT_MODE [] = { 8,00,   12,00,   00,00,  04,00,00,00 };
uint8_t SET_NORMAL_MODE [] = { 8,00,   12,00,   00,00,  64,00,00,00 };
uint8_t SET_DEBUG_MODE  [] = { 8,00,   12,00,   00,00,  00,00,00,00 };

#define ELEMENTS(x) (sizeof(x)/sizeof(x[0]))

// comment out below for simple passthru
#define LOOK4SYNC   // reverse eng of S3KM1110

// do not enable when driven by the S3K app 
//#define ECHO_TELNET_TO_SERIAL

// define on linux side only 
#define TEST_GET_FIRMWARE
#define TEST_REPORT_MODE
#define TEST_NORMAL_MODE
#define TEST_DEBUG_MODE

void request_firmware(void);


//---------------------------------------------------------

// danger uart 1 is the flash (ouch).
HardwareSerial HostSide(0);

uint32_t lastReading = 0;

void rs232_setup(void)
{
    HostSide.begin(115200);

	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, 1);
	
    sk3k1110.setPins(RX2, TX2);  // runtime overide
    sk3k1110.begin(115200);

    HostSide.printf("used for interept of serial stream for S3KM11110\n");

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
int sync1StartRight = 0;

int sync1EndLeft = 0;
int sync1EndRight = 0;
bool bSync1FoundLeft = false;
bool bSync1FoundRight = false;

int sync2EndLeft = 0;
int sync2EndRight = 0;
bool bSync2FoundLeft = false;
bool bSync2FoundRight = false;


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
	HostSide.printf("%s\ns", what);
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
		static uint16_t cnt;
		static bool bToggle;
		
		uint8_t achar;
		achar = sk3k1110.read();

		cnt++;
		if (cnt > 30)
		{
			cnt = 0;
			bToggle = !bToggle;
			digitalWrite(PIN_LED, bToggle);
		}		
	
#ifdef LOOK4SYNC

		//HostSide.print(achar, HEX); HostSide.print("  ");

		if (!bSync1FoundRight)
		{
			if (achar == SYNC_START1[sync1StartRight])
			{
				if (sync1StartRight == 3)
				{
				  Right[indexRight++] = achar;
				  bSync1FoundRight = true;
				  sync1EndRight = 0;
				}
				else
				{
				  digitalWrite(PIN_LED, 1);
				  sync1StartRight++;
				  Right[indexRight++] = achar;
				}
			}
			else
			{
				//sync still not found. or sync miss
				sync1StartRight = 0;
				indexRight = 0;
				bSync1FoundRight = false;
				digitalWrite(PIN_LED, 0);
			}
		}
		else
		{
			// sync has been found... collect payload
			Right[indexRight++] = achar;

			if (achar == SYNC_END1[sync1EndRight])
			{
				if (sync1EndRight == 3)
				{
					sync1EndRight = 0;
					
					dump("FROM SK3:", Right, MIN(MAX_CHARS, indexRight));
					indexRight = 0;
					sync1StartRight = 0;
					bSync1FoundRight = false;
				}
				else
				{
					digitalWrite(PIN_LED, 1);
					sync1EndRight++;
				}
			}
		}
#endif
	  HostSide.write(achar);  // not print!
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
	
	if (!bSync1FoundLeft)
	{
		if (achar == SYNC_START1[syncStartLeft])
		{
		  if (syncStartLeft == 3)
		  {
			  Left[indexLeft++] = achar;
			  bSync1FoundLeft = true;
			  sync1EndLeft = 0;
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
		  bSync1FoundLeft = false;
		  digitalWrite(PIN_LED, 0);
		}
	}
  	else
	{
		// sync has been found... collect payload
		Left[indexLeft++] = achar;

		if (achar == SYNC_END1[sync1EndLeft])
		{
			if (sync1EndLeft == 3)
			{
				sync1EndLeft = 0;

				
				dump("TO SK3:", Left, MIN(MAX_CHARS, indexLeft));
				indexLeft = 0;
				syncStartLeft = 0;
				bSync1FoundLeft = false;
			}
			else
			{
				digitalWrite(PIN_LED, 1);
				sync1EndLeft++;
			}
		}
	}
#endif
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

//--------------------------------------------------------------

void request_normal(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START1); i++) 
	{
		toS3(SYNC_START1[i]);
	}

	for (i = 0; i < ELEMENTS(SET_NORMAL_MODE); i++) 
	{
		toS3(SET_REPORT_MODE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END1); i++) 
	{
		toS3(SYNC_END1[i]);
	}
}

//--------------------------------------------------------------

void request_debug(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START1); i++) 
	{
		toS3(SYNC_START1[i]);
	}

	for (i = 0; i < ELEMENTS(SET_DEBUG_MODE); i++) 
	{
		toS3(SET_DEBUG_MODE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END1); i++) 
	{
		toS3(SYNC_END1[i]);
	}
}


//--------------------------------------------------------------

void request_report(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START1); i++) 
	{
		toS3(SYNC_START1[i]);
	}

	for (i = 0; i < ELEMENTS(SET_REPORT_MODE); i++) 
	{
		toS3(SET_REPORT_MODE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END1); i++) 
	{
		toS3(SYNC_END1[i]);
	}
}


//-------------------------------------------------------------

void request_firmware(void)
{
	int i;
	for (i = 0; i < ELEMENTS(SYNC_START1); i++) 
	{
		toS3(SYNC_START1[i]);
	}

	for (i = 0; i < ELEMENTS(GET_FIRMWARE); i++) 
	{
		toS3(GET_FIRMWARE[i]);
	}
	for (i = 0; i < ELEMENTS(SYNC_END1); i++) 
	{
		toS3(SYNC_END1[i]);
	}
}

//----------------------------------------------------------
static int testRequestCnt =0;
static int testReport = 0;

void rs232_loop() 
{

#ifdef TEST_GET_FIRMWARE
  if (testRequestCnt++ > 10000)
  {
  	request_firmware();
  	testRequestCnt = 0;
  }
#endif

#ifdef TEST_REPORT_MODE
  if (testReport++ == 50000)
  {
  	request_report();  // one shot.
  }
#endif

#ifdef TEST_NORMAL_MODE
	if (testReport++ == 50000)
	{
	  request_normal();  // one shot.
	}
#endif

#ifdef TEST_DEBUG_MODE
	  if (testReport++ == 50000)
	  {
		request_debug();  // one shot.
	  }
#endif
  

  process_toS3();
  process_fromS3();
}

