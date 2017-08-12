#pragma once

/* ROBO LIMB CAN PROTOCOL 
 *
 * Baud Rate 1Mbit/sec
 * BitTime=10,TSEG1=6,TSEG2=1,SamplingPoint 80%
 *
 * 
*/

#define ID_SERIAL_NUM_READ	0x402
#define ID_SERIAL_NUM_ANS	0x403
#define ID_CONFIG			0x404

#define ID_THUMB			0x101
#define ID_INDEX			0x102
#define ID_MIDDLE			0x103
#define ID_RING				0x104
#define ID_LITTLE			0x105
#define ID_ROTATOR			0x106

#define ID_THUMB_STATE		0x201
#define ID_INDEX_STATE		0x202
#define ID_MIDDLE_STATE		0x203
#define ID_RING_STATE		0x204
#define ID_LITTLE_STATE		0x205
#define ID_ROTATOR_STATE	0x206

#define ID_QUICK_GRIP		0x301

#define COMM_STOP				0
#define COMM_CLOSE				1
#define COMM_OPEN				2

// Digit->state Value
#define STATE_STOP				0
#define STATE_CLOSING			1
#define STATE_OPENING			2
#define STATE_STALLED_CLOSED	3
#define STATE_STALLED_OPEN		4
#define STATE_THUMB_ROT_NOT_EDGE	0
#define STATE_THUMB_ROT_EDGE		1

// digit[ XXXX ]
#define THUMB		0
#define INDEX		1
#define	MIDDLE		2
#define RING		3
#define LITTLE		4
#define ROTATOR		5

#define MODE_QUICK	0
#define MODE_PWM	1

