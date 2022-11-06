#include <stdio.h>
#include <string.h>
#include <string>
#include <errno.h>
#include <chrono>
#include <thread>

#include <wiringPi.h>
#include <wiringSerial.h>

volatile double target_temp = 73;   //TO DO: GET THIS FROM USER
volatile double current_temp;

volatile double target_pH = 7.1;  //TO DO: GET THIS FROM USER
volatile double current_pH;
static std::string tx_packet;
 
/*struct tx_packet{
	volatile double temp = target_temp;
	volatile double pH = target_pH;
};
*/
struct rx_packet{
	volatile double temp = current_temp;
	volatile double pH = current_pH;

};


using namespace std;


int main ()
{
  using namespace std::this_thread;
  using namespace std::chrono;
	
  int serial_port ;
  char dat;
  if ((serial_port = serialOpen("/dev/ttyS0", 9600)) < 0)	/* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  while(1){
	  
	if(serialDataAvail (serial_port) )
	{ 
		dat = serialGetchar (serial_port);		/* receive character serially*/	
		printf ("%c", dat);
		//sleep_for(seconds(1));
		fflush (stdout);
		tx_packet = "Target temperature: " 
			+ to_string(target_temp) 
			+ " Target pH value: " 
			+ to_string(target_pH) 
			+ "\n";
		for (size_t i = 0; i < tx_packet.size(); ++i){
		  serialPutchar(serial_port, tx_packet[i]);		/* transmit character serially on port */
	    }
	}
  }

}
