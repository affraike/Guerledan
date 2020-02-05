#include "ros/ros.h"
#include "encoders_boat/Message_encoders.h"

//SOURCE Serial : https://github.com/xanthium-enterprises/Serial-Port-Programming-on-Linux/blob/master/USB2SERIAL_Read/Reciever%20(PC%20Side)/SerialPort_read.c

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>  
#include <errno.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <ctime>  
#include <math.h>
using namespace std;
 
#define IMU_Port "/dev/ttyUSB0"

int readNext(int fd)
{
	char read_buffer[2];
	int  bytes_read = read(fd,&read_buffer,1);
	int value = (int)read_buffer[0];
	return value;
}

bool checkStart(int a, int b)
{
	if(a == 255 and b == 10)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "Node_encoders");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<encoders_boat::Message_encoders>("encoders", 1000);
ros::Rate loop_rate(2500);

int fd;
fd = open(IMU_Port,O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   *//* O_RDWR   - Read/Write access to serial port       */	/* O_NOCTTY - No terminal will control the process   *//* Open in blocking mode,read will wait              */
							
							                                        
							
if(fd == -1)						/* Error Checking */
	   cout <<"Failure to connect to : " <<IMU_Port << endl;
else
	   cout <<"Connected to : " <<IMU_Port <<endl;


/*---------- Setting the Attributes of the serial port using termios structure --------- */

struct termios SerialPortSettings;	/* Create the structure                          */

tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

/* Setting the Baud rate */
cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

/* 8N1 Mode */
SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

/* Setting Time outs */
SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 10 characters */
SerialPortSettings.c_cc[VTIME] = 1; /* Wait indefinetly   */


if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    cout << "ERROR ! in Setting attributes" << endl;
else
    cout <<"Attributes OK" <<endl;
	
    /*------------------------------- Read data from serial port -----------------------------*/

tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
cout << "Reading DATA" << endl;

int last_char = -1;
int current_char = -1;
int compteur = -1;

int time_1 = -1;
int time_2 = -1;
int time_3 = -1;
int time_4 = -1; 

int enc1_v1 = -1;
int enc1_v2 = -1;
int enc2_v1 = -1;
int enc2_v2 = -1;

int value_sync = -1;
bool sync = false;


while(ros::ok())
{
int v1 = readNext(fd);
current_char = v1;
if(checkStart(last_char,current_char))
{
	if(time_1 != -1 and time_2 != -1 and time_3 != -1 and time_4 != -1 and enc1_v1 != -1 and enc1_v2 != -1 and enc2_v1 != -1 and enc2_v2 != -1 and value_sync == 170 and sync == true)
	{
		int current_value_v1 = enc1_v1*256+enc1_v2;
		int current_value_v2 = enc2_v1*256+enc2_v2;
		double current_time = 256.*256.*256.*time_1 + 256.*256.*time_2 + 256.*time_3 + time_4;
		encoders_boat::Message_encoders msg;
		msg.time = current_time;
		msg.enc_left = current_value_v2; 
		msg.enc_right = current_value_v1;

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();	
		//cout <<current_time <<" "<< current_value_v1 <<" "<<current_value_v2 << endl;
	}
	compteur = 1;
	enc1_v1 = -1;
    enc1_v2 = -1;
    enc2_v1 = -1;
    enc2_v2 = -1;
    time_1 = -1;
	time_2 = -1;
	time_3 = -1;
	time_4 = -1;
    value_sync = -1;
    sync = false;
}
else
{
	compteur++;
	if(compteur == 2)
	{
		time_1 = current_char;
	}
	else if(compteur == 3)
	{
		time_2 = current_char;
	}
	else if(compteur == 4)
	{
		time_3 = current_char;
	}
	else if(compteur == 5)
	{
		time_4 = current_char;
	}
	else if(compteur == 8)
	{
		enc1_v1 = current_char;
	}
	else if(compteur == 9)
	{
		enc1_v2 = current_char;
	}
	else if(compteur == 10)
	{
		enc2_v1 = current_char;
	}
	else if(compteur == 11)
	{
		enc2_v2 = current_char;
	}
	else if(compteur == 16)
	{
		value_sync = current_char;
		sync = true;
	}
}
last_char = current_char;
usleep(1000);
}
close(fd); /* Close the serial port */
return 0;
}