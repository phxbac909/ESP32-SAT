#ifndef SOCKET_H
#define SOCKET_H

extern const int SERVER_PORT;
extern const char* TARGET_IP;
extern const int TARGET_PORT;

void startSocketTasks();
void serverTask(void *parameter);

#endif