
uint8 STATUS_FINISHED_SUCCESS = 1
uint8 STATUS_FINISHED_ERROR = 2
uint8 STATUS_IN_PROGRESS = 3
uint8 STATUS_TIMEOUT = 4
uint8 STATUS_CONNECTION_ERROR = 5

string host
string remote

uint16 remote_port

uint32 call_id
string service
uint8 status
