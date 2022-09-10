#pragma once

#include <unistd.h>
#include <string>
#include <vector>

#define READ_ONLY_FD 0
#define WRITE_ONLY_FD 1

class BashCommandWrapper
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

    BashCommandWrapper() {}

    void execBashCommandWithPipes();
    int execBashCommandWithPipes_LBL();
    // void operator()(std::string _stdin, std::string _cmd)
    // {
    //     this->StdIn = _stdin;
    //     this->Command = _cmd;
    //     this->execBashCommandWithPipes();
    // }

    int delimittedCopyPipeContents(BashCommandWrapper::PipeWrapper roPipe, std::vector<std::string> _delimittedData);

private:
    PipeWrapper stdInPipe;
    PipeWrapper stdOutPipe;
    PipeWrapper stdErrPipe;

    std::vector<std::string> delimittedData;
    static bool readLineFromPipe(PipeWrapper roPipe, std::string &dest);
    void createPipes();
    int createFork();
    void copyPipeContents(PipeWrapper roPipe, std::string &dest);
    
    void cleanUp();
};