#!/usr/bin/sudo bash
# Nipun Dhananjaya Weerakkodi <nipun.dhananjaya@gmail.com>

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

HELP="
Enable SLCAN (USB-to-CAN) adapters as CAN network interface

-------------------------------------------------------------------------------------------------------------
Make sure to follow steps below to run this script without password:
    - move the scrip to /home/<USER NAME> directory
    - Change the script's ownership to root and make the script executable:
        $ sudo chown root:root /home/$USER/low_battery_ditector.bash
        $ sudo chmod 700 /home/$USER/low_battery_ditector.bash
    - Set up sudoers to allow the script to be executed witout requiring a password:
        * Type 'sudo visudo' command at the terminal to open the sudo permissions (sudoers) file.
        * after the line '%sudo ALL=(ALL:ALL) ALL' add the below line. where 'username' is your username:
            username ALL=(ALL) NOPASSWD: /home/username/slcan.bash

-------------------------------------------------------------------------------------------------------------
Options:
    -s<X> (Where X is a number in rane [0, 8]; default: 8)
        Set CAN bitrate to:
        0 - 10 Kbps
        1 - 20 Kbps
        2 - 50 Kbps
        3 - 100 Kbps
        4 - 125 Kbps
        5 - 250 Kbps (Cyphal/CAN recommended)
        5 - 250 Kbps (Cyphal/CAN recommended)
        6 - 500 Kbps (Cyphal/CAN recommended)
        7 - 800 Kbps
        8 - 1   Mbps (Cyphal/CAN recommended, default)

    -S<X> (Where X is baudrate o serial port of slcan device, default: 1000000)
        Configure baud rate to use on the interface.
        This option is mostly irrelevant for USB adapters.
    
    -t<X> (Where X is txqueuelen of CAN interface, defaul: 1000)

    -r
        Remove all SLCAN interfaces.

    --can-name <X> (Where X is a name for CAN interface, default: can0)

    --dev-path <X> (Where X is USB device path of SLLCAN device, default: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0)
        Relevent device path id for your device can be found in the dir. of /dev/serial/by-id/
--------------------------------------------------------------------------------------------------------------
"

if [ "$1" == '--help' ] || [ "$$1" == '-h' ]; then echo "$HELP"; exit; fi

# To terminate
function die() { echo $@ >&2; exit 1; }

# Check whether can-utils is installed. if not terminate
which slcand > /dev/null || die "can-utils is NOT found. 
Please install can-utils using 'sudo apt-get install can-utils'".

# --------------------------------------------------

function stop_slcand() {
    printf "[slcan.bash]${CYAN}Stopping slcand...${NC}\n" >&2
    # Trying to close with SIGINT first
    killall -INT slcand &> /dev/null
    sleep 0.3
    # If SIGINIT didn't close slcand, Then trying the defaul signal, which is SIGSTREAM
    slcand_kill_retries=10
    while killall slcand &> /dev/null
    do
        (( slcand_kill_retries -= 1 ))
        [[ "$slcand_kill_reties" > 0 ]] || die "Failed to stop slcand"
        sleep 1
    done
    echo "slcand is stopped"
}

function initialize_slcan() {
    printf "[slcan.bash]${CYAN}Attaching '$DEV_PATH' to '$CAN_IFACE'...${NC}\n" >&2
    printf "[slcan.bash]${CYAN}SLCAN parameters: | bitrate_code: '$BITRATE_CODE' | txqueuelen: $TXQUEUELEN | baudrate: '$BAUDRATE' |${NC}\n" >&2
    printf "[slcan.bash]${CYAN}Running command to create slcan interface: slcand -o -c -s$BITRATE_CODE -S$BAUDRATE $DEV_PATH $CAN_IFACE${NC}\n" >&2
    slcand -o -c -s$BITRATE_CODE -S$BAUDRATE $DEV_PATH $CAN_IFACE || return 5
    sleep 2
    printf "[slcan.bash]${CYAN}Running command to up $CAN_IFACE: ip link set $CAN_IFACE up txqueuelen $TXQUEUELEN${NC}\n" >&2
    ip link set $CAN_IFACE up txqueuelen $TXQUEUELEN || return 6
}


CAN_IFACE='can0'
DEV_PATH='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BITRATE_CODE=8 # 1Mbit/sec
TXQUEUELEN=1000
BAUDRATE=1000000 # serial device baudrate

init_can=1

next_option=''
while [ -n "$1" ]; do
    case $1 in
    -r | --remove)
        stop_slcand
        init_can=0
        ;;

    -s[0-8])
        BITRATE_CODE=${1:2}
        ;;
    
    -S*)
        BAUDRATE=${1:2}
        ;;

    -t*)
        TXQUEUELEN=${1:2}
        ;;
    --*)
        next_option=${1:2}
        ;;
    
    -*)
        die "Invalid option: $1"
        ;;
    
    *) 
        if   [ "$next_option" = 'can-name' ]; then CAN_IFACE=$1
        elif [ "$next_option" = 'dev-path' ]; then DEV_PATH=$1
        fi
        next_option=''
        ;;
    esac
    shift
done

[ "$next_option" = '' ] || die "Expected argument for option '$next_option'"

if [ $init_can -eq 1 ]
then 
    initialize_slcan
    printf "[slcan.bash]${GREEN}$CAN_IFACE is initialized${NC}\n" >&2
fi

ip link show can0
is_can_initialized=$?
exit $is_can_initialized

