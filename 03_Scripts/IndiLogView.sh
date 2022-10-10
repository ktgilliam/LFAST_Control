#!/bin/bash

log_dir=$"~/.indi/logs"

#logArray=($(ls -ltrR "$log_dir" | egrep '\.log$'))
logArray=($(find ~/.indi/logs/ | egrep '\.log$'))


#echo "$logArray"
( IFS=$'\n'; echo "${logArray[*]}" )


