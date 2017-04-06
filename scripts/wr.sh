#!/bin/bash

DIR=/home/dmrl-3/DMRL_3/dev/GlobalRep/dmrl_3/src/antenna/ethercat-1.5.2/tool
DRIVE=$1
MODE=$2

if [ x"" = x"$DRIVE" ]; then
    echo "Empty 'drive' parameter. Bailout..."
    exit 1
fi

if [ x"" = x"$MODE" ]; then
    echo "Empty mode. Bailout..."
    exit 1
fi

conf_mode() {
        local drive=$1
        local mode=$2

        if ((drive == 0)); then
                if ((mode == 1)); then # <= 1 deg
                        $DIR/ethercat -p 0 download -t uint16 0x2100 0 1500
                        $DIR/ethercat -p 0 download -t uint16 0x2101 0 1
                        $DIR/ethercat -p 0 download -t uint16 0x2102 0 1
                        $DIR/ethercat -p 0 download -t uint16 0x2103 0 0
                        $DIR/ethercat -p 0 download -t uint16 0x2104 0 0
                        $DIR/ethercat -p 0 download -t uint16 0x2105 0 0
                        $DIR/ethercat -p 0 download -t uint16 0x2106 0 550
                        $DIR/ethercat -p 0 download -t uint16 0x2107 0 850
                        $DIR/ethercat -p 0 download -t uint16 0x2108 0 10
                        $DIR/ethercat -p 0 download -t uint16 0x2109 0 6
                        $DIR/ethercat -p 0 download -t uint16 0x210A 0 0
                        $DIR/ethercat -p 0 download -t uint16 0x210B 0 0
                        $DIR/ethercat -p 0 download -t uint16 0x2301 0 1
                        $DIR/ethercat -p 0 download -t uint16 0x2302 0 1
                        $DIR/ethercat -p 0 download -t uint16 0x2303 0 1
                        $DIR/ethercat -p 0 download -t uint16 0x2304 0 0
                        $DIR/ethercat -p 0 download -t uint32 0x6081 0 200000
                        $DIR/ethercat -p 0 download -t uint32 0x6083 0 35000
                        $DIR/ethercat -p 0 download -t uint32 0x6084 0 35000
                elif ((mode == 10)); then # <= 10 deg
                        $DIR/ethercat -p 0 download -t uint16 0x2100 0 1000
                        $DIR/ethercat -p 0 download -t uint16 0x2106 0 500
                        $DIR/ethercat -p 0 download -t uint16 0x2107 0 800
                        $DIR/ethercat -p 0 download -t uint16 0x2108 0 25
                        $DIR/ethercat -p 0 download -t uint16 0x2109 0 15
                        $DIR/ethercat -p 0 download -t uint32 0x6081 0 200000
                        $DIR/ethercat -p 0 download -t uint32 0x6083 0 10000
                        $DIR/ethercat -p 0 download -t uint32 0x6084 0 10000
                elif ((mode == 90)); then # <= 90 deg
                        $DIR/ethercat -p 0 download -t uint16 0x2100 0 800
                        $DIR/ethercat -p 0 download -t uint16 0x2106 0 550
                        $DIR/ethercat -p 0 download -t uint16 0x2107 0 850
                        $DIR/ethercat -p 0 download -t uint16 0x2108 0 25
                        $DIR/ethercat -p 0 download -t uint16 0x2109 0 15
                        $DIR/ethercat -p 0 download -t uint32 0x6081 0 200000
                        $DIR/ethercat -p 0 download -t uint32 0x6083 0 50000
                        $DIR/ethercat -p 0 download -t uint32 0x6084 0 50000
                else
                        echo "Invalid mode: $mode. Bailout..."
                        exit 1
                fi
        elif ((drive == 1)); then
                 if ((mode == 5)); then # <= 5 deg ???
                        $DIR/ethercat -p 1 download -t uint16 0x2100 0 2000
                        $DIR/ethercat -p 1 download -t uint16 0x2101 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2102 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2103 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2104 0 98
                        $DIR/ethercat -p 1 download -t uint16 0x2105 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2106 0 100
                        $DIR/ethercat -p 1 download -t uint16 0x2107 0 150
                        $DIR/ethercat -p 1 download -t uint16 0x2108 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2109 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x210A 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x210B 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2301 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2302 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2303 0 10
                        $DIR/ethercat -p 1 download -t uint16 0x2304 0 0
                        $DIR/ethercat -p 1 download -t uint32 0x6081 0 200000
                        $DIR/ethercat -p 1 download -t uint32 0x6083 0 100000
                        $DIR/ethercat -p 1 download -t uint32 0x6084 0 100000
                elif ((mode == 100)); then
                        $DIR/ethercat -p 1 download -t uint16 0x2100 0 200
                        $DIR/ethercat -p 1 download -t uint16 0x2101 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2102 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2103 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2104 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2105 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2106 0 800
                        $DIR/ethercat -p 1 download -t uint16 0x2107 0 1200
                        $DIR/ethercat -p 1 download -t uint16 0x2108 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2109 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x210A 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x210B 0 0
                        $DIR/ethercat -p 1 download -t uint32 0x6081 0 200000
                        $DIR/ethercat -p 1 download -t uint32 0x6083 0 100000
                        $DIR/ethercat -p 1 download -t uint32 0x6084 0 100000
                else
                        echo "Invalid mode: $mode. Bailout..."
                        exit 1
                fi
        else
            echo "Invalid drive: $drive. Bailout..."
        fi
}

echo "Write drive=$DRIVE mode=$MODE"
conf_mode $DRIVE $MODE
echo "Done"
