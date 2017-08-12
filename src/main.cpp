#include <canlib.h>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "robolimb/robolimb_can_protocol.h"
#include "robolimb/RobolimbState.h"

#define READ_WAIT_INFINITE    (unsigned long)(-1)

robolimb::RobolimbState hand;

char *endPtr = NULL;
int channel = 0;
canHandle hnd;
canStatus stat;

int16_t cmdBuf[7];
bool readFlag;

void callback(const std_msgs::Int16MultiArray::ConstPtr& arr);
static void check(char* id, canStatus stat);

int main(int argc, char** argv)
{
	ros::init(argc,argv,"robolimb");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("robolimb_cmd", 10, callback); 
	ros::Publisher pub = nh.advertise<robolimb::RobolimbState>("robolimb_state", 1);
	ros::Rate loop_rate(1000);

	long id;
	unsigned char msgin[4]={0,};
	unsigned int dlc;
	unsigned int flag;
	unsigned long time;
	int16_t *u16p = (int16_t*)&msgin[2];
	uint8_t cmdPre;
	int idx;

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

		char config_msg[4]={0x00,0x00,0x00,0xFF};

		stat = canWrite(hnd, ID_CONFIG,config_msg , 4, canMSG_STD);
		check("canWrite",stat);

		if( stat != canOK )
			goto ErrorExit;

//		config_msg[3]=0;
//		canWrite(hnd, ID_QUICK_GRIP, config_msg, 4, canMSG_STD );
	}

	ROS_INFO("CAN Comm with ROBOLIMB Established!");

	while( ros::ok() )
	{
		if( readFlag )
		{
			char msgBuf[4] = { 0,};

			// Quick Grip Mode
			if( cmdBuf[0] == MODE_QUICK )
			{
				msgBuf[3] = (uint8_t)cmdBuf[6];

				if( msgBuf[3] != cmdPre )
				{
					canWrite(hnd, ID_QUICK_GRIP, msgBuf, 4, canMSG_STD);
					cmdPre = msgBuf[3];
				}
			}	
			else // PWM Grip Mode
			{
				for(int i = 1; i < 7; i++)
				{
					if( cmdBuf[i] < 0 )
					{
						msgBuf[1] = COMM_OPEN;
						cmdBuf[i] = -1 * cmdBuf[i];
					}
					else if( cmdBuf[i] == 0 )
						msgBuf[1] = COMM_STOP;
					else
						msgBuf[1] = COMM_CLOSE;

					if(cmdBuf[i] < 10 && cmdBuf[i] != 0 )
						cmdBuf[i] = 10;

					msgBuf[3] = (uint8_t)(cmdBuf[i] & 0x00FF);
					msgBuf[2] = (uint8_t)((cmdBuf[i] & 0xFF00)>>8);  
				
					canWrite(hnd, ID_THUMB+i-1, msgBuf, 4, canMSG_STD);
				}
			}

			ROS_INFO("<CMD> %d %d %d %d %d %d %d",cmdBuf[0],cmdBuf[1],cmdBuf[2],cmdBuf[3],cmdBuf[4],cmdBuf[5],cmdBuf[6]);
			readFlag = false;
		}

		
		stat = canRead(hnd, &id, &msgin, &dlc, &flag, &time);

		switch( id )
		{
			case ID_THUMB_STATE:
			case ID_INDEX_STATE:
			case ID_MIDDLE_STATE:
			case ID_RING_STATE:
			case ID_LITTLE_STATE:
			case ID_ROTATOR_STATE:
			
				idx = id - 0x201;
				u16p = (int16_t*)&msgin[2];
				hand.timestamp = time;
				hand.thumb_rot_edge = msgin[0];
				hand.state[idx] = msgin[1];
				hand.cur[idx] = (float)(*u16p/16)/21825;
				break;
		}

		if( stat == canOK )
		{
			pub.publish( hand ); 
		}

		loop_rate.sleep();
		ros::spinOnce();
	}


ErrorExit:
	
	stat = canBusOff(hnd);
	check("canBusOff", stat);
	stat = canClose(hnd);
	check("canClose", stat);

	return 0;
}


void callback(const std_msgs::Int16MultiArray::ConstPtr& arr)
{
	if( readFlag ) return;

	int i = 0;

	for(std::vector<int16_t>::const_iterator it = arr->data.begin(); it != arr->data.end(); ++it )
	{
		cmdBuf[i] = *it;
		i++;
	}

	readFlag = true;

	return;
}

static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}
