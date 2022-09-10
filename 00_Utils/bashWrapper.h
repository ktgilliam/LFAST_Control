#pragma once

#include <unistd.h>
#include <string>

#define READ_ONLY_FD 0
#define WRITE_ONLY_FD 1

class BashWrapper
{
public:
    int ExitStatus = 0;
    std::string Command;
    std::string StdIn;
    std::string StdOut;
    std::string StdErr;

    struct PipeWrapper
    {
        int fileDescriptors[2];
        void close();
        void close(int rw);
    };

    BashWrapper() {}

    void execBashCommandWithPipes();

    // void operator()(std::string _stdin, std::string _cmd)
    // {
    //     this->StdIn = _stdin;
    //     this->Command = _cmd;
    //     this->execBashCommandWithPipes();
    // }

private:
    PipeWrapper stdInPipe;
    PipeWrapper stdOutPipe;
    PipeWrapper stdErrPipe;

    void createPipes();
    int createFork();
    void copyPipeContents(PipeWrapper roPipe, std::string &dest);
    ssize_t readLineFromPipe(PipeWrapper roPipe, std::string &dest);
    void cleanUp();
};