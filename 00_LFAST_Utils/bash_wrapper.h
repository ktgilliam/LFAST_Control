// /** 
//     \file lfast_mount_driver.h
//     \brief LFAST Az/El Pedestal INDI Driver
//     \author Kevin Gilliam
// */

#pragma once

#include <string>
// #include <iostream>

class BashWrapper
{
public:
    BashWrapper(){};
    std::string execBashCommand(const std::string cmd, int &out_exitStatus);

    class CommandWithPipes
    {
    public:
        int ExitStatus = 0;
        std::string Command;
        std::string StdIn;
        std::string StdOut;
        std::string StdErr;

        void execBashCommandWithPipes();
    };
};