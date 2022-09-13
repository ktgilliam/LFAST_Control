#include "bash_wrapper.h"
#include <array>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <iostream>
#include <bits/stdc++.h>

void BashCommand::execBashCommandWithPipes()
{
    // This class based on example from: https://dev.to/aggsol/calling-shell-commands-from-c-8ej

    // To hijack the stdin/stdout pipes of our commands, we implement our own and overwrite them
    this->createPipes();
    // Have the system create a second process for the child, and we will pass it the data from the parent
    int status = this->createFork();

    copyPipeContents(stdOutPipe, StdOut);
    copyPipeContents(stdErrPipe, StdErr);

    if (WIFEXITED(status))
    {
        ExitStatus = WEXITSTATUS(status);
    }

    cleanUp();
}

void BashCommand::cleanUp()
{
    stdInPipe.close();
    stdOutPipe.close();
    stdErrPipe.close();
}

void BashCommand::createPipes()
{
    // Create pipes for stdin, stdout, and stderr. pipe() returns -1 if there was an error creating the pipe.

    // Man page for ::pipe(): https://www.man7.org/linux/man-pages/man2/pipe.2.html
    // Background info on pipes: https://brandonwamboldt.ca/how-linux-pipes-work-under-the-hood-1518/

    // Create the stdin pipe
    auto rc = ::pipe(stdInPipe.fileDescriptors);
    if (rc < 0)
    {
        throw std::runtime_error(std::strerror(errno));
    }

    // Create the stdout pipe
    rc = ::pipe(stdOutPipe.fileDescriptors);
    if (rc < 0)
    {
        // Close the stdin pipe that was just made
        stdInPipe.close();
        throw std::runtime_error(std::strerror(errno));
    }

    // Create the stderr pipe
    rc = ::pipe(stdErrPipe.fileDescriptors);
    if (rc < 0)
    {
        // Close the stdin and stdout pipes that were just made
        stdInPipe.close();
        stdOutPipe.close();
        throw std::runtime_error(std::strerror(errno));
    }
}

int BashCommand::createFork()
{
    auto pid = ::fork(); // Create a fork and get the process ID.
    // Note that the child process is initially a duplicate of this process, so this code
    // Needs to check to find out if it is the parent or child process.
    //
    // If fork is successful, fork() returns the new PID to the parent, and returns 0 to the child
    // otherwise, fork() returns -1 to the parent.

    // The fork failed to be created.
    if (pid < 0)
    {
        cleanUp();
        throw std::runtime_error("Failed to fork");
    }

    if (pid > 0) // This is the parent
    {
        // The parent is the one which calls the command (not the command itself)
        stdInPipe.close(READ_ONLY_FD);   // Parent does not read from stdin (nothing is being piped to the parent, we are doing that manually)
        stdOutPipe.close(WRITE_ONLY_FD); // Parent does not write to stdout (because we are stealing it)
        stdErrPipe.close(WRITE_ONLY_FD); // Parent does not write to stderr (because we are also stealing that)

        // The parent process's stdin pipe's write end
        if (::write(stdInPipe.fileDescriptors[WRITE_ONLY_FD], StdIn.data(), StdIn.size()) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }
        stdInPipe.close(WRITE_ONLY_FD); // Done writing
    }

    else if (pid == 0) // This is the new child
    {
        // Make stdin/stdout/stderr point to the same file descriptor as the parent

        ::dup2(stdInPipe.fileDescriptors[READ_ONLY_FD], STDIN_FILENO); // Make the child's stdin read from the parent's stdin
        stdInPipe.close(WRITE_ONLY_FD);                                // Child does not write to stdin

        ::dup2(stdOutPipe.fileDescriptors[WRITE_ONLY_FD], STDOUT_FILENO); // Make the child's stdout write to the parent's stdout
        stdOutPipe.close(READ_ONLY_FD);                                   // Child does not read from stdout

        ::dup2(stdErrPipe.fileDescriptors[WRITE_ONLY_FD], STDERR_FILENO); // Make the child's stdout write to the parent's stdout
        stdErrPipe.close(READ_ONLY_FD);                                   // Child does not read from stderr

        // Send the bash command to the system
        ::execl("/bin/bash", "bash", "-c", Command.c_str(), nullptr);
        ::exit(EXIT_SUCCESS);
    }

    int status = 0;
    ::waitpid(pid, &status, 0);
    return status;
}

void BashCommand::copyPipeContents(BashCommand::PipeWrapper roPipe, std::string &dest)
{
    // Create a temporary buffer for storing data read from the pipe's read-only file descriptor
    std::array<char, 256> buffer;

    ssize_t bytes = 0;
    do
    {
        bytes = ::read(roPipe.fileDescriptors[READ_ONLY_FD], buffer.data(), buffer.size());
        dest.append(buffer.data(), bytes);
    } while (bytes > 0);
}

void BashCommand::PipeWrapper::close()
{
    ::close(fileDescriptors[READ_ONLY_FD]);
    ::close(fileDescriptors[WRITE_ONLY_FD]);
}

void BashCommand::PipeWrapper::close(int rw)
{
    ::close(fileDescriptors[rw]);
}

std::vector<std::string> BashCommand::splitByDelimeter(std::string s, std::string delimeter)
{
    size_t pos = 0;
    std::string token;
    std::vector<std::string>  tokens;
    while ((pos = s.find(delimeter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        // std::cout << token << std::endl;
        tokens.push_back(token);
        s.erase(0, pos + delimeter.length());
    }
    tokens.push_back(s);

    // std::stringstream check(s);  
    // std::string intermediate;
     
    // // Tokenizing w.r.t. space ' '
    // while(getline(check, intermediate, ' '))
    // {
    //     tokens.push_back(intermediate);
    // }

    return tokens;
}
