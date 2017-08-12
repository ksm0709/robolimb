#include <canlib.h>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Uint16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "robolimb/robolimb_can_protocol.h"

#define READ_WAIT_INFINITE    (unsigned long)(-1)

using namespace std;

queue<MsgCan> msgQue;

Hand hand;

char *endPtr = NULL;
int channel = 0;
canHandle hnd;
canStatus stat;

uint16_t cmdBuf[7];
char msg[4]={0x00,0x00,0x00,0x00};


void callback(const std_msgs::Uint16MultiArray::ConstPtr& arr);
int canUnpack(const char *buf);

static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%d)\n", id, (int)stat, buf);
  }
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"robolimb");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("robolimb_cmd", 1, callback); 
	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robolimb_stat", 1);
	ros::Rate loop_rate(10);
	
	MsgCan *msgBuf;

	if( argc > 1 )
		channel = strtol(argv[1], &endPtr, 10);
	{
		/* Open channels, parameters and go on bus */
		hnd = canOpenChannel(channel, canOPEN_EXCLUSIVE);
		if (hnd < 0) {
			printf("canOpenChannel %d", channel);
			check("", (canStatus)hnd);
			return -1;
		}

		stat = canSetBusParams(hnd,canBITRATE_1M, 6, 1, 2, 0, 0);
		check("canSetBusParams", stat);
		if (stat != canOK) {
			goto ErrorExit;
		}

		stat = canBusOn(hnd);
		check("canBusOn", stat);
		if (stat != canOK) {
			goto ErrorExit;
		}	
	}
	
	while( ros::ok() )
	{
		if( !msgQue.empty() )
		{
			msgBuf = &msgQue.front();		

			stat = canWrite(hnd, msgBuf->id, msgBuf->byte, 4, canMSG_STD);
			check("canWrite",stat);

			if( stat != canOK )
				goto ErrorExit;
		}

//		stat = canWrite(hnd, ID_SERIAL_NUM_READ, msg, 4, canMSG_STD);
//		check("canWrite",stat);
//
//		if( stat == canOK )
//		{
//			ROS_INFO("<WRITE> CH:%d , ID:0x%X , MSG : 0x%x 0x%x 0x%x 0x%x", channel, ID_SERIAL_NUM_READ, msg[0],msg[1],msg[2],msg[3] );
//		}
//		else
//		{
//			goto ErrorExit;
//		}

		long id;
		unsigned char msgi[4]={0,};
		unsigned int dlc;
		unsigned int flag;
		unsigned long time;
		stat = canReadWait(hnd, &id, &msgi, &dlc, &flag, &time, 100);

		check("canReadWait",stat);

		if( stat == canOK )
		{
			ROS_INFO("<READ> CH:%d , ID:0x%X , MSG : 0x%x 0x%x 0x%x 0x%x(%d)\n", channel, id, msgi[0],msgi[1],msgi[2],msgi[3] ,dlc);
		}
		else
			goto ErrorExit;

		loop_rate.sleep();
//		stat = canWriteSync(hnd,1000);
//		check("canWriteSync", stat);
//
//		if( stat != canOK )
//		{
//			goto ErrorExit;
//		}
	}


ErrorExit:
	
	stat = canBusOff(hnd);
	check("canBusOff", stat);
	stat = canClose(hnd);
	check("canClose", stat);

	return 0;
}


void callback(const std_msgs::Uint16MultiArray::ConstPtr& arr)
{
	int i = 0;

	for(std::vector<uint16_t>::const_iterator it = arr->data.begin(); it != arr->data.end(); ++it )
	{
		cmdBuf[i] = *it;
		i++;
	}

	if( i != 7 ) return;

	MsgCan msgBuf;

	memset( msgBuf, 0, sizeof(MsgCan) );

	// Quick Grip Mode
	if( cmdBuf[0] == MODE_QUICK )
	{
		msgBuf.id = 0x301;
		msgBuf.[3] = (uint8_t)cmdBuf[6];

		msgQue.push_back( msgBuf );
	}	
	else // PWM Grip Mode
	{
		for(i = 1; i < 7; i++)
		{
			msgBuf.id = 0x100+i;

			if( cmdBuf[i] < 0 )
			{
				msgBuf.byte[1] = COMM_OPEN;
				cmdBuf[i] = -1 * cmdBuf[i];
			}
			else if( cmdBuf[i] == 0 )
				msgBuf.byte[1] = COMM_STOP;
			else
				msgBuf.byte[1] = COMM_CLOSE;

			msgBuf.byte[2] = (uint8_t)((cmdBuf[i]>>8) & 0x00FF);  
			msgBuf.byte[3] = (uint8_t)(cmdBuf[i] & 0x00FF);

			msgQue.push_back( msgBuf );
		}
	}

	return;
}
