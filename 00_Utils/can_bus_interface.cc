
#include "can_bus_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <iostream>
#include <algorithm>

#include <bash_wrapper.h>
#include <stdexcept>


bool CanBusComms::checkInterfaceExists(std::string ifName)
{
	BashCommand bc;
	bc.Command = R"(ip link show | grep -P '^\d+:' | sed 's/://g' | awk '{print $2}')";
	bc.execBashCommandWithPipes();
	bool exists = false;

	std::vector<std::string> ifList = BashCommand::splitByDelimeter(bc.StdOut, "\n");
	// for(int ii = 0; ii < ifList.size(); ii++)
	// {
	// 	std::cout<< "###" << ii << " " << ifList.at(ii) << std::endl;
	// }
	if (std::find(ifList.begin(), ifList.end(), ifName) != ifList.end())
	{
		exists=true;
	}
	return exists;
}

int CanBusComms::openCanSocket(std::string &ifName)
{
	if(!checkInterfaceExists(ifName))
	{
		throw std::runtime_error("Interface");
		return 1;
	}
	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		throw std::runtime_error("Socket");
		return 1;
	}


	strcpy(ifr.ifr_name, ifName.c_str());
	ioctl(sock, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		throw std::runtime_error("Bind");
		return 1;
	}
	return 0;
}

int CanBusComms::closeCanSocket()
{
	if (::close(sock) < 0)
	{
		throw std::runtime_error("Socket");
		std::perror("Close");
		return 1;
	}
	return 0;
}

void CanBusComms::printCanMessage(struct can_frame &frame)
{
	std::cout << std::hex << frame.can_id << "#" << frame.can_dlc;

	for (int ii = 0; ii < frame.can_dlc; ii++)
	{
		std::cout << std::hex << frame.data[ii] << " " ;
	}

	std::cout << std::endl;
}

int CanBusComms::canReceiveNonBlocking(struct can_frame &frame)
{
	int nbytes;

	std::cout << "# CAN Sockets Receive Demo:" << std::endl;

	nbytes = ::read(sock, &frame, sizeof(struct can_frame));

	if (nbytes < 0)
	{
		throw std::runtime_error("Read error");
		return 1;
	}
	return 0;
}

int CanBusComms::canTransmit(unsigned int id, char *data, unsigned int nBytes)
{
	struct can_frame frame;
	frame.can_id = id;
	frame.can_dlc = 5;

	if(nBytes > 8)
	{
		throw std::runtime_error("Payload g.t. 8 bytes.");
	}

	memcpy(frame.data, data, nBytes);

	if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
	{
		throw std::runtime_error("Write Failed.");
		return 1;
	}

	return 0;
}

int CanBusComms::setCanFilter(int id, int mask)
{
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	struct can_filter rfilter[1];

	rfilter[0].can_id = id;
	rfilter[0].can_mask = mask;
	// rfilter[1].can_id   = 0x200;
	// rfilter[1].can_mask = 0x700;

	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	return 0;
}
