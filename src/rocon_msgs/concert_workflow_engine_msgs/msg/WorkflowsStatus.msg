#pre-define workflow status
int8 READY = 1
int8 START = 2
int8 RUNNING = 3
int8 STOP = 4
int8 ERROR = -1

string service_name # service name to launch workflow
int8 status # status of workflow
string message # user friendly message