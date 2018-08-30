#ifndef COMDEFINES_H_

#define COMDEFINES_H_

#define LOG_DEBUG

#define SYNCH_MSG 0x0EFFFFFFF

#define SYNCH_MSG_ACK 0x0FE0000FF

#define COM_ERR_DESYNCH_NUM 4

#define CMD_NOT_SYNCHED -1

#define CMD_SYNCHED 0

#define CMD_LENGTH_MAX 0xFF

/////////////////////////////////
// MASTER Command's Identifiers
/////////////////////////////////

// Master Orders: from 0x00 to 0x1F
/////////////////////////////////

#define CMDM_START_CTL			0x00
#define CMDM_STOP_CTL			0x01
#define CMDM_UPDATE				0x02
#define CMDM_SET_REF			0x03
#define CMDM_SEL_SCHEME			0x04
#define CMDM_IS_INDEXED			0x05
#define CMDM_SAFE_ACK			0x06

// Set Safe Bounds: from 0x20 to 0x4F
/////////////////////////////////

#define CMDM_SAFE_START			0x20

#define CMDM_SAFE_POS_MAX		0x20
#define CMDM_SAFE_POS_MIN		0x21

#define CMDM_SAFE_SPEED_MAX		0x22
#define CMDM_SAFE_SPEED_MIN		0x23

#define CMDM_SAFE_PWM_MAX		0x24
#define CMDM_SAFE_PWM_MIN		0x25

#define CMDM_SAFE_CURR_MAX		0x26
#define CMDM_SAFE_CURR_MIN		0x27

#define CMDM_SAFE_FORCE_MAX		0x28
#define CMDM_SAFE_FORCE_MIN		0x29

#define CMDM_SAFE_POS_E_MAX		0x2A
#define CMDM_SAFE_POS_E_MIN		0x2B

#define CMDM_SAFE_SPEED_E_MAX	0x2C
#define CMDM_SAFE_SPEED_E_MIN	0x2D

#define CMDM_SAFE_CURR_E_MAX	0x2E
#define CMDM_SAFE_CURR_E_MIN	0x2F

#define CMDM_SAFE_FORCE_E_MAX	0x30
#define CMDM_SAFE_FORCE_E_MIN	0x31

// Set Ctl Pars: from 0x50 to 0x7E
/////////////////////////////////

#define KP_OFFSET 0x0
#define KI_OFFSET 0x1
#define KD_OFFSET 0x2
#define ISAT_OFFSET 0x3
#define SAT_OFFSET 0x4


#define CMDM_PAR_SET_START		0x50

#define CMDM_BREAK_PARS_START   0x50

#define CMDM_SET_BREAK_KP		0x50
#define CMDM_SET_BREAK_KI		0x51
#define CMDM_SET_BREAK_KD		0x52
#define CMDM_SET_BREAK_ISAT		0x53
#define CMDM_SET_BREAK_SAT		0x54

#define CMDM_FORCE_PARS_START   0x55

#define CMDM_SET_FORCE_KP		0x55
#define CMDM_SET_FORCE_KI		0x56
#define CMDM_SET_FORCE_KD		0x57
#define CMDM_SET_FORCE_ISAT		0x58
#define CMDM_SET_FORCE_SAT		0x59

#define CMDM_SPEED_PARS_START   0x5A

#define CMDM_SET_SPEED_KP		0x5A
#define CMDM_SET_SPEED_KI		0x5B
#define CMDM_SET_SPEED_KD		0x5C
#define CMDM_SET_SPEED_ISAT		0x5D
#define CMDM_SET_SPEED_SAT		0x5E

#define CMDM_POS_PARS_START   0x5F

#define CMDM_SET_POS_KP			0x5F
#define CMDM_SET_POS_KI			0x60
#define CMDM_SET_POS_KD			0x61
#define CMDM_SET_POS_ISAT		0x62
#define CMDM_SET_POS_SAT		0x63

#define CMDM_SET_BREAK_INT_FUN	0x64
///
#define INT_FUN_LINEAR			0x01
#define INT_FUN_ARCTG			0x02
///

#define CMDM_SET_BREAK_T_CHANGE	0x65

#define CMDM_SET_SYS_FS			0x66

#define CMDM_SET_QEI_MAX_CNT	0x67

#define CMDM_IDX_PARS_START   0x68

#define CMDM_SET_IDX_KP			0x68
#define CMDM_SET_IDX_KI			0x69
#define CMDM_SET_IDX_KD			0x6A
#define CMDM_SET_IDX_ISAT		0x6B
#define CMDM_SET_IDX_SAT		0x6C

#define CMDM_SET_DB_POS_PWM		0x6D
#define CMDM_SET_DB_NEG_PWM		0x6E

#define CMDM_SET_K_POS_PWM		0x6F
#define CMDM_SET_K_NEG_PWM		0x70

#define CMDM_PING				0x7F

// Ref Identifiers
/////////////////////////////////

#define REF_ID_PWM_D	((char) -1)
#define REF_ID_CURR_D	((char) -2)
#define REF_ID_POS_D	((char) -3)
#define REF_ID_SPEED_D	((char) -4)
#define REF_ID_FORCE_D	((char) -5)
#define REF_ID_FORCE	((char) -6)
#define REF_ID_PWM_CMP	((char) -7)

/////////////////////////////////
// SLAVE (Tiva) Command's Identifiers
/////////////////////////////////

// Enumerated Commands (<0)
/////////////////////////////////

#define CMDS_CTL_STARTED		0x00
#define CMDS_CTL_STOPPED		0x01
#define CMDS_UPDATING			0x02
#define CMDS_SET_REF			0x03
#define CMDS_SCHEME_SELECTED	0x04


#define CMDS_ERR_SAFE_POS		0x05
#define CMDS_ERR_SAFE_SPEED		0x06
#define CMDS_ERR_SAFE_PWM		0x07
#define CMDS_ERR_SAFE_CURR		0x08
#define CMDS_ERR_SAFE_SPEED_E	0x09
#define CMDS_ERR_SAFE_POS_E		0x0A

#define CMDS_ERR_LIMIT_SWITCH	0x0B

#define CMDS_ERR_SAFE_FORCE		0x0C
#define CMDS_ERR_SAFE_FORCE_E	0x0D

#define CMDS_ERR_SEL_SCH		0x0E

#define CMDS_ERR_SAFE_CURR_E	0x0F

#define CMDS_PAR_SET			0x10

#define CMDS_SYS_RUNNING		0x12

#define CMDS_ERR_REF_NOT_FOUND	0x13

#define CMDS_ERR_SAFE_NOT_FOUND	0x14

#define CMDS_ERR_BAD_CMD		0x15

#define CMDS_ERR_CTL_MOD_NOT_FOUND	0x16

#define CMDS_MOTOR_OFF			0x17
#define CMDS_IDX				0x18

#define CMDS_SAFE_ACK			0x19

#define CMDS_PING				CMDM_PING

// Signal Identifiers
/////////////////////////////////

#define SIG_ID_POS		((char)	(-1))
#define SIG_ID_SPEED	((char)	(-2))
#define SIG_ID_PWM		((char)	(-3))
#define SIG_ID_CURR		((char) (-4))
#define SIG_ID_FORCE	((char) (-5))

// Safety Identifiers
/////////////////////////////////

#define SAFE_ID_POS		CMDS_ERR_SAFE_POS
#define SAFE_ID_SPEED	CMDS_ERR_SAFE_SPEED
#define SAFE_ID_PWM		CMDS_ERR_SAFE_PWM
#define SAFE_ID_CURR	CMDS_ERR_SAFE_CURR
#define SAFE_ID_CURR_E	CMDS_ERR_SAFE_CURR_E
#define SAFE_ID_SPEED_E	CMDS_ERR_SAFE_SPEED_E
#define SAFE_ID_POS_E	CMDS_ERR_SAFE_POS_E
#define SAFE_ID_FORCE_E	CMDS_ERR_SAFE_FORCE_E
#define SAFE_ID_FORCE	CMDS_ERR_SAFE_FORCE
#define SAFE_ID_ERR		0x00

// Ctls ID
/////////////////////////////////

#define SYS_SCHEMES_NUM		0x05

#define CTL_ID_BREAK		0x00
#define CTL_ID_POS			0x01
#define CTL_ID_SPEED		0x02
#define CTL_ID_FORCE		0x03
#define CTL_ID_INDEX		0x04

// Ctl Modules ID
/////////////////////////////////

#define CTLM_ID_PWM			0x00
#define CTLM_ID_PID_BRAKE	0x01
#define CTLM_ID_PID_FORCE	0x02
#define CTLM_ID_PID_POS		0x03
#define CTLM_ID_PID_SPEED	0x04
#define CTLM_ID_CONT_BREAK	0x05
#define CTLM_ID_FORCE_D_IN	0x06
#define CTLM_ID_FORCE_IN	0x07
#define CTLM_ID_POS_D_IN	0x08
#define CTLM_ID_SPEED_D_IN	0x09
#define CTLM_ID_SPEED_SENS	0x0A
#define CTLM_ID_POS_SENS	0x0B
#define CTLM_ID_CURR_SENS	0x0C
#define CTLM_ID_SUM_POS		0x0D
#define CTLM_ID_SUM_SPEED	0x0E

#endif
