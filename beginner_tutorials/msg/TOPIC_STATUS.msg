uint8 S_OUT_OF_SYNCH = 0
uint8 S_IDLE = 1
uint8 S_UPDATE_SYS = 2
uint8 S_SELECT_SCHEME = 3
uint8 S_START_SCHEME = 4
uint8 S_RUNNING = 5
uint8 S_STOP_SCHEME = 6
uint8 S_MONITOR_BREAK = 7
uint8 S_UNSAFE = 8
uint8 S_SET_PAR = 9
uint8 S_UNINDEXED = 10

uint8 status

bool index

uint8 ERR_SAFE_POS = 0
uint8 ERR_SAFE_SPEED = 1
uint8 ERR_SAFE_PWM = 2
uint8 ERR_SAFE_CURR = 3
uint8 ERR_SAFE_SPEED_E = 4
uint8 ERR_SAFE_POS_E = 5
uint8 ERR_SAFE_FORCE = 6
uint8 ERR_SAFE_FORCE_E = 7
uint8 ERR_SAFE_CURR_E = 8

uint8 ERR_LIMIT_SWITCH = 9

uint8 safeErrId
bool safe

uint8 ERR_SET_PAR = 0
uint8 ERR_BAD_CMD = 1

uint8 errId
bool err
