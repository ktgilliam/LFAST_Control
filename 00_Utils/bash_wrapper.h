// /** 
//     \file lfast_mount_driver.h
//     \brief LFAST Az/El Pedestal INDI Driver
//     \author Kevin Gilliam
//
//     \details Code adapted from published example here: 
//              https://dev.to/aggsol/calling-shell-commands-from-c-8ej
// */

#pragma once

#include <string>

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