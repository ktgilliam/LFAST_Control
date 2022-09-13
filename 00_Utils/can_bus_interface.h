
#pragma once
#include <string>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CanBusComms
{
public:
    int openCanSocket(std::string &ifName);
    int canReceiveNonBlocking(struct can_frame &frame);
    int canTransmit(unsigned int id, char *data, unsigned int nBytes);
    int closeCanSocket();
    static bool checkInterfaceExists(std::string ifName);
    static void printCanMessage(struct can_frame &frame);
    int setCanFilter(int id, int mask);
private:
    int sock;
    struct ifreq ifr;
    struct sockaddr_can addr;
};


int testCanReceive();
int testCanTransmit();