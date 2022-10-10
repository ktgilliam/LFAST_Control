#!/bin/bash

scriptName=$0

function print_usage() {
	echo "usage: sudo bash $scriptName [-{s|k}] [-b<baud>]"
	echo "	-s Start serial CAN bus"
	echo "	-k Kill serial CAN bus"
	echo "	-b <baud>"
	return
}


function checkOptions() {
	declare -i START_KILL_ARG_CHECK=0
	local OPTIND
	while getopts ":skb:" opt ; do
	 case "$opt" in 
		 s)START_KILL_ARG_CHECK=$((START_KILL_ARG_CHECK+1));;
		 k)START_KILL_ARG_CHECK=$((START_KILL_ARG_CHECK+1));;
		 b) ;;
		 \?) echo "WHYYYYYY";;
	 esac
	done
	
	if [[ START_KILL_ARG_CHECK -ne 1 ]]; then
		echo "Error: Must use -s to start OR -k to kill."
		exit
	fi
	if [ "$EUID" -ne 0 ] || [ $# -eq 0 ]; then
		print_usage
		exit
	fi
}
	
checkOptions "$@"

#This script configures the Karbon's internal microcontroller to be controlled with can-utils
#For debugging:
#trap '[[ $BASH_COMMAND != echo* ]] && [[ $BASH_COMMAND != read* ]] && echo $BASH_COMMAND' DEBUG

# ASCII Command vs CAN Bitrate
# s0 --- s1 --- s2 --- s3 --- s4 --- s5 --- s6 --- s7 --- s8
# 10     20     50     100    125    250    500    800    1000  Kbits/s
declare -i BAUD=8


# Detect correct port for device interfaces
TERM_PORT=$(ls -l /dev/serial/by-id/ | grep 1fc9_00a3 | sed 's/.*\///g' | awk '{if(NR==2) print $0}')
CANB_PORT=$(ls -l /dev/serial/by-id/ | grep 1fc9_00a3 | sed 's/.*\///g' | awk '{if(NR==1) print $0}')
# For each device listed | look for ones with 1fc9_00a3 | trim the lines down to the device label | if and only if there are two results, return the first (for TERM_PORT)/second (for CANB_PORT).

if [ -z "$TERM_PORT" ] || [ -z "$CANB_PORT" ]; then
	ISKARBON=0
	DEV=slcan0
else
	ISKARBON=1
	# Name of slcan device to attach
	DEV=can0
fi




# Start (s) or stop/kill (k) the service
while getopts ":skb:" opt; do
    case "$opt" in
    	b)
    		BAUD=${OPTARG}
    		#re_isanum='^[0-9]+$'

		#if ! [ $BAUD =~ ^[0-8]+$ ] || (($BAUD < 9 ) && ( $BAUD > 0 ))
		if [ ${BAUD} -ge 9 ] || [ ${BAUD} -lt 0 ] ; then
			echo "BAUD INVALID"
			echo "# Baud Select:    0 ---- 1 ---- 2 ---- 3 ---- 4 ---- 5 ---- 6 ---- 7 ---- 8"
			echo "# Rate [kbits/s]: 10     20     50     100    125    250    500    800    1000"
			read -p "Select baudrate option:" BAUD_SELECT
			BAUD=$BAUD_SELECT
		fi
    		;;
        k)
		# Stop the SLCAN service and turn off the can device
		echo "Shutting down can interface..."
		#if [ $ISKARBON -eq 1 ]; then
			echo "step 1"
			sudo ifconfig $DEV down 2> /dev/null
			sudo pkill slcand
			echo "step 2"
			
			sudo slcand -c /dev/"$CANB_PORT"
			sudo pkill slcand
		#else
			
		#fi
		;;
        s)
		# Open the can device, and start the slcan service.
		if [ $ISKARBON -eq 1 ]; then
			echo "Terminal interface on: $TERM_PORT"

			# Set the mode to slcan
			echo -ne "set can-mode slcan" > /dev/$TERM_PORT
		else
			echo "Karbon uC not detected. Select available device to use for CAN bus:"
			AVAILABLE_USB=$(ls -l /dev/serial/by-id/ | grep USB | sed 's/.*\///g')
			IFS=$'\n'
			select serialDevice in $AVAILABLE_USB; do 
				CANB_PORT=$serialDevice
				break;	
			done
		fi
		echo "CAN Bus  interface on: $CANB_PORT"
		echo "Attaching slcan device..."

		# Attach to the port with slcand
		sudo slcand -s$BAUD -o /dev/"$CANB_PORT" $DEV

		# Give interface time to come up
		sleep .25

		# Enable the inteface
		sudo ifconfig $DEV up
		sudo ifconfig $DEV txqueuelen 1000

		echo "Interface is set up."
		;;
         :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;

    esac
    
	if [ "$EUID" -ne 0 ] || [ $# -eq 0 ]; then
		echo "Argument -s or -k must be provided (but not both)"
	fi
done


