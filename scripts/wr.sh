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

read_reg() {
    local reg_type=$1

}

# SINT Signed 8-bit -128 ~127
# USINT Unsigned 8-bit 0 ~ 255
# INT Signed 16-bit -32768 ~ 32767
# UINT Unsigned 16-bit 0 ~ 65535
# DINT Signed 32-bit -21247483648 ~ 21247483647
# UDINT Unsigned 32-bit 0 ~ 4294967295
# STRING The String Value


# args:
#   $0 - data type
#   $1 - address
#   $2 - subindex
#   $3 - description
read_reg() {
    local data_type=$1
    local address=$2
    local subindex=$3
    local description=$4

    local app_data_type=""
    case "$data_type" in
        "SINT")    app_data_type="int8"     ;;
        "USINT")   app_data_type="uint8"    ;;
        "INT")     app_data_type="int16"    ;;
        "UINT")    app_data_type="uint16"   ;;
        "DINT")    app_data_type="int32"    ;;
        "UDINT")   app_data_type="uint32"   ;;
        "STRING")  app_data_type="string"   ;;
        *)
            echo "*Error*: invalid data-type $data_type"
            exit 1
    esac

    for ((i = 0; i < 2; i++)); do
        show_label "$i: $address:$subindex, $description [$app_data_type]"
        $DIR/ethercat -m 0 upload -p $i --type $app_data_type $address $subindex
    done
}


wr() {
    $DIR/ethercat -p 0 download $@
}

conf_mode() {
        local drive=$1
        local mode=$2

        if ((drive == 0)); then
                if ((mode == 1)); then # <= 1 deg
                        $DIR/ethercat -p 0 download -t uint16 0x2100 0 4000
                        $DIR/ethercat -p 0 download -t uint16 0x2106 0 200
                        $DIR/ethercat -p 0 download -t uint16 0x2107 0 350
                        $DIR/ethercat -p 0 download -t uint16 0x2108 0 40
                        $DIR/ethercat -p 0 download -t uint16 0x2109 0 25
                        $DIR/ethercat -p 0 download -t uint32 0x6081 0 6000
                        $DIR/ethercat -p 0 download -t uint32 0x6083 0 3000
                        $DIR/ethercat -p 0 download -t uint32 0x6084 0 3000
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
                        $DIR/ethercat -p 0 download -t uint32 0x6081 0 50000
                        $DIR/ethercat -p 0 download -t uint32 0x6083 0 50000
                        $DIR/ethercat -p 0 download -t uint32 0x6084 0 50000
                else
                        echo "Invalid mode: $mode. Bailout..."
                        exit 1
                fi
        elif ((drive == 1)); then
                 if ((mode == 55555then # <= 5 deg ???
                        $DIR/ethercat -p 1 download -t uint16 0x2100 0 2000
                        $DIR/ethercat -p 1 download -t uint16 0x2101 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2102 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2103 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2104 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2105 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x2106 0 100
                        $DIR/ethercat -p 1 download -t uint16 0x2107 0 150
                        $DIR/ethercat -p 1 download -t uint16 0x2108 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x2109 0 1
                        $DIR/ethercat -p 1 download -t uint16 0x210A 0 0
                        $DIR/ethercat -p 1 download -t uint16 0x210B 0 0
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


my_conf_mode() {
    local mode=$1

    if ((mode == 1)); then # <= 1 deg
        inertia_ratio_setting=4000
        speed_proportional_gain_1=100
        speed_proportional_gain_2=350
        speed_integral_time_constant_1=40
        speed_integral_time_constant_2=25
        profile_speed=90000
        profile_acceleration=90000
        profile_decleration=90000
    elif ((mode == 10)); then # <= 10 deg
        inertia_ratio_setting=1000
        speed_proportional_gain_1=500
        speed_proportional_gain_2=800
        speed_integral_time_constant_1=25
        speed_integral_time_constant_2=15
        profile_speed=200000
        profile_acceleration=10000
        profile_decleration=10000
    elif ((mode == 90)); then # <= 90 deg
        inertia_ratio_setting=800
        speed_proportional_gain_1=550
        speed_proportional_gain_2=850
        speed_integral_time_constant_1=25
        speed_integral_time_constant_2=15
        profile_speed=50000
        profile_acceleration=50000
        profile_decleration=50000
    else
        echo "Invalid mode: $mode. Bailout..."
        exit 1
    fi

    $DIR/ethercat -p 0 download -t uint16 0x2100 0 $inertia_ratio_setting
    $DIR/ethercat -p 0 download -t uint16 0x2106 0 $speed_proportional_gain_1
    $DIR/ethercat -p 0 download -t uint16 0x2107 0 $speed_proportional_gain_2
    $DIR/ethercat -p 0 download -t uint16 0x2108 0 $speed_integral_time_constant_1
    $DIR/ethercat -p 0 download -t uint16 0x2109 0 $speed_integral_time_constant_2
    $DIR/ethercat -p 0 download -t uint32 0x6081 0 $profile_speed
    $DIR/ethercat -p 0 download -t uint32 0x6083 0 $profile_acceleration
    $DIR/ethercat -p 0 download -t uint32 0x6084 0 $profile_decleration
}



echo "Write drive=$DRIVE mode=$MODE"
conf_mode $DRIVE $MODE
echo "Done"
