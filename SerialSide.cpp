/*
  SerialPassthrough sketch
*/

#include "ESPTelnet.h"

//-------------------------------------------
#define MIN(a,b)  (a < b) ? a : b

// give hardware serial a heads up we are not standard                                     
#define RX2 18
#define TX2 19

extern ESPTelnet telnet; 

#include <HardwareSerial.h>
HardwareSerial sk3k1110(2);

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
	
    sk3k1110.setPins(RX2, TX2);  // runtime overide
    sk3k1110.begin(115200);

    HostSide.printf("used for interept of serial stream for S3KM11110\n");

    while (HostSide.available()) HostSide.read();
    while (sk3k1110.available()) sk3k1110.read();

    HostSide.printf("looping\n");
}

#define MAX_CHARS 80

uint8_t left[MAX_CHARS];
uint8_t right[MAX_CHARS];

int indexLeft = 0;
int indexRight = 0;


char bigBuffer[2000];

void sendit(char *what)
{
    telnet.println(what);
	HostSide.printf("%s", what);
}


#define WIDE 16
void dump(char * tag, uint8_t *array, int size)
{

	sprintf(bigBuffer, "%s ==== %d =======\n", tag, size);
	sendit(bigBuffer);
	
	while (size > 0)
	{
		bigBuffer[0] = 0;
		
		int run = MIN(size, WIDE);
		int len = 0;
		
		for( int i = 0; i < run; i++)
		{
			if (i < run)
				len += sprintf(&bigBuffer[len], "%02X ", array[i]);
			else
				len += sprintf(&bigBuffer[len], "   ", array[i]);
		}
		
		len += sprintf(&bigBuffer[len], "===");

		for( int i = 0; i < run; i++)
		{
            
			len += sprintf(&bigBuffer[len], "%c", array[i] < 0x20 ? '.': array[i] > 0x7F? '_' : array[i]);
		}
		
		len += sprintf(&bigBuffer[len], "\n");

		sendit(bigBuffer);
		
		size -= run;
	}

}

void rs232_loop() 
{
  if (HostSide.available())
  {
    uint8_t achar;
    achar = HostSide.read(); 

	left[indexLeft++] = achar;
	if (indexLeft == MAX_CHARS || achar == 0xA)
	{
		dump("TO SK3:", left, MIN(MAX_CHARS, indexLeft -1));
		indexLeft = 0;
	}

    sk3k1110.write(achar);  // read it and send it out HostSide1 (pins 0 & 1)
  }

  if (sk3k1110.available()) 
  {
	  uint8_t achar;
	  achar = sk3k1110.read();
	  
	  right[indexRight++] = achar;
	  if (indexRight == MAX_CHARS || achar == 0xA)
	  {
		  dump("FromS3 :", right, MIN (MAX_CHARS, indexRight-1));
		  indexRight = 0;
	  }
	  

    HostSide.write(achar);  // read it and send it out HostSide (USB)
  }
}


