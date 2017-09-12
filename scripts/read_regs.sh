#!/bin/bash

ETHERCAT=/home/dmrl-3/DMRL_3/dev/GlobalRep/dmrl_3/src/antenna/ethercat-1.5.2/tool/ethercat

REGS=(
    0x1000
    0x1001
    0x1008
    0x1009
    0x100A

    0x2000
    0x2001
    0x2002
    0x2003
    0x2004
    0x2005
    0x2006
    0x2007
    0x2008
    0x2009
    0x200A
    0x200B
    0x200C
    0x200D
    0x200E
    0x200F
    0x2010
    0x2011
    0x2012
    0x2013
    0x2014
    0x2015
    0x2016
    0x2017
    0x2018
    0x2019
    0x201A
    0x201B
    0x201C
    0x201D
    0x201E
    0x201F
    0x2020

    0x2100
    0x2101
    0x2102
    0x2103
    0x2104
    0x2105
    0x2106
    0x2107
    0x2108
    0x2109
    0x210A
    0x210B
    0x210C
    0x210D
    0x210E
    0x210F
    0x2110
    0x2111
    0x2112
    0x2113
    0x2114
    0x2115
    0x2116
    0x2117
    0x2118
    0x2119
    0x211A
    0x211B
    0x211C
    0x211D
    0x211E
    0x211F
    0x2120

    0x2300
    0x2301
    0x2302
    0x2303
    0x2304
    0x2305
    0x2306
    0x2307
    0x2308
    0x2309
    0x230A
    0x230B
    0x230C
    0x230D
    0x230E
    0x230F
    0x2310
    0x2311

    0x2400
    0x2401
    0x2402
    0x2403
    0x2404
    0x2405
    0x2406
    0x2407
    0x2408
    0x2409
    0x240A
    0x240B
    0x240C
    0x240D
    0x240E

    0x2500
    0x2501
    0x2502
    0x2503
    0x2504
    0x2505
    0x2506
    0x2507
    0x2508
    0x2509
    0x250A
    0x250B
    0x250C
    0x250D
    0x250E
    0x250F
    0x2510
    0x2511
    0x2512
    0x2513
    0x2514
    0x2515
    0x2516
    0x2517
    0x2518
    0x2519

    0x605A
    0x605B
    0x605C
    0x605D
    0x6072
    0x6076
    0x607D
    0x6085
    0x6087
    0x6091
    0x6098
    0x60B0
    0x60B1
    0x60B2
    0x60B8
    0x60B9
    0x60E0
    0x60E1
    0x60FD
    0x60FE
)

INDICES=(
    [0x607D]=2
    [0x6091]=2
    [0x60FE]=2
)

TYPES=(
    [0x1000]=uint32
    [0x1001]=uint8
    [0x1008]=string
    [0x1009]=string
    [0x100A]=string

    [0x2000]=uint16
    [0x2001]=uint16
    [0x2002]=uint32
    [0x2003]=uint16
    [0x2004]=uint16
    [0x2005]=uint16
    [0x2006]=uint16
    [0x2007]=uint16
    [0x2008]=uint16
    [0x2009]=uint16
    [0x200A]=uint16
    [0x200B]=uint16
    [0x200C]=uint16
    [0x200D]=uint16
    [0x200E]=uint16
    [0x200F]=uint16
    [0x2010]=uint16
    [0x2011]=uint16
    [0x2012]=uint16
    [0x2013]=uint16
    [0x2014]=int16
    [0x2015]=int16
    [0x2016]=int16
    [0x2017]=int16
    [0x2018]=uint16
    [0x2019]=uint16
    [0x201A]=uint16
    [0x201B]=uint16
    [0x201C]=uint16
    [0x201D]=uint16
    [0x201E]=uint16
    [0x201F]=uint16
    [0x2020]=uint16
    
    [0x2100]=uint16
    [0x2101]=uint16
    [0x2102]=uint16
    [0x2103]=uint16
    [0x2104]=uint16
    [0x2105]=uint16
    [0x2106]=uint16
    [0x2107]=uint16
    [0x2108]=uint16
    [0x2109]=uint16
    [0x210A]=uint16
    [0x210B]=uint16
    [0x210C]=uint16
    [0x210D]=uint16
    [0x210E]=uint16
    [0x210F]=uint16
    [0x2110]=uint16
    [0x2111]=uint16
    [0x2112]=uint16
    [0x2113]=uint16
    [0x2114]=uint16
    [0x2115]=uint16
    [0x2116]=uint16
    [0x2117]=uint16
    [0x2118]=uint16
    [0x2119]=uint16
    [0x211A]=uint16
    [0x211B]=uint16
    [0x211C]=uint16
    [0x211D]=uint16
    [0x211E]=uint16
    [0x211F]=uint16
    [0x2120]=uint16

    [0x2300]=int16
    [0x2301]=uint16
    [0x2302]=uint16
    [0x2303]=uint16
    [0x2304]=int16
    [0x2305]=int16
    [0x2306]=int16
    [0x2307]=int16
    [0x2308]=uint16
    [0x2309]=uint16
    [0x230A]=uint16
    [0x230B]=uint16
    [0x230C]=int16
    [0x230D]=uint16
    [0x230E]=uint16
    [0x230F]=uint16
    [0x2310]=uint16
    [0x2311]=uint16

    [0x2400]=uint16
    [0x2401]=uint16
    [0x2402]=uint16
    [0x2403]=uint16
    [0x2404]=uint16
    [0x2405]=uint16
    [0x2406]=uint16
    [0x2407]=uint16
    [0x2408]=uint16
    [0x2409]=uint16
    [0x240A]=uint16
    [0x240B]=uint16
    [0x240C]=int32
    [0x240D]=string
    [0x240E]=uint16

    [0x2500]=uint16
    [0x2501]=uint16
    [0x2502]=uint16
    [0x2503]=uint16
    [0x2504]=uint16
    [0x2505]=uint16
    [0x2506]=uint16
    [0x2507]=uint16
    [0x2508]=uint16
    [0x2509]=uint16
    [0x250A]=uint16
    [0x250B]=uint16
    [0x250C]=uint16
    [0x250D]=uint16
    [0x250E]=uint16
    [0x250F]=uint16
    [0x2510]=uint16
    [0x2511]=uint16
    [0x2512]=uint16
    [0x2513]=uint16
    [0x2514]=uint16
    [0x2515]=uint16
    [0x2516]=uint16
    [0x2517]=uint16
    [0x2518]=uint16
    [0x2519]=uint16

    [0x605A]=int16
    [0x605B]=int16
    [0x605C]=int16
    [0x605D]=int16
    [0x6072]=uint16
    [0x6076]=uint32
    [0x607D]=int32
    [0x6085]=uint32
    [0x6087]=uint32
    [0x6091]=int32
    [0x6098]=int8
    [0x60B0]=int32
    [0x60B1]=int32
    [0x60B2]=int16
    [0x60B8]=uint16
    [0x60B9]=uint16
    [0x60E0]=uint16
    [0x60E1]=uint16
    [0x60FD]=uint32
    [0x60FE]=uint32
)

NAMES=(
    [0x1000]="Device Type"
    [0x1001]="Error Register"
    [0x1008]="Device Name"
    [0x1009]="Hardware Version"
    [0x100A]="Software Version"

    [0x2000]="Motor ID"
    [0x2001]="Encoder Type"
    [0x2002]="Encoder Pulse per Revolution"
    [0x2003]="Node ID"
    [0x2004]="Rotation Direction Setting"
    [0x2005]="Absolute Encoder Configuration"
    [0x2006]="Main Power Fail Check Mode"
    [0x2007]="Main Power Fail Check Time"
    [0x2008]="7SEG Display Selection"
    [0x2009]="Brake Resistor Configuration"
    [0x200A]="Brake Resistor Derating Factor"
    [0x200B]="Brake Resistor Value"
    [0x200C]="Brake Resistor Power"
    [0x200D]="Brake Resistor Peak Power"
    [0x200E]="Brake Resistor Peak Power Duration"
    [0x200F]="Overload Check Base"
    [0x2010]="Overload Warning Level"
    [0x2011]="PWM Off Delay Time"
    [0x2012]="Dynamic Brake Control Mode"
    [0x2013]="Emergency Stop Configuration"
    [0x2014]="Warning Mask Configuration"
    [0x2015]="U Phase Current Offset"
    [0x2016]="V Phase Current Offset"
    [0x2017]="W Phase Current Offset"
    [0x2018]="Magnetic Pole Pitch"
    [0x2019]="Linear Scale Resolution"
    [0x201A]="Commutation Method"
    [0x201B]="Commutation Current"
    [0x201C]="Commutation Time"
    [0x201D]="Grating Period of Sinusoidal Encoder"
    [0x201E]="Homing Done Behaviour"
    [0x201F]="Velocity Function Select"
    [0x2020]="Motor and Hall Phase Correction"

    [0x2100]="Inertia Ratio"
    [0x2101]="Position Loop Gain 1"
    [0x2102]="Speed Loop Gain 1"
    [0x2103]="Speed Loop Integral Time Constant 1"
    [0x2104]="Torque Command Filter Time Constant 1"
    [0x2105]="Position Loop Gain 2"
    [0x2106]="Speed Loop Gain 2"
    [0x2107]="Speed Loop Integral Time Constant 2"
    [0x2108]="Torque Command Filter Time Constant 2"
    [0x2109]="Position Command Filter Time Constant"
    [0x210A]="Position Command Average Filter Time Constant"
    [0x210B]="Speed Feedback Filter Time Constant"
    [0x210C]="Velocity Feed-forward Gain"
    [0x210D]="Velocity Feed-forward Filter Time Constant"
    [0x210E]="Torque Feed-forward Gain"
    [0x210F]="Torque Feed-forward Filter Time Constant"
    [0x2110]="Torque Limit Function Setting"
    [0x2111]="External Positive Torque Limit Value"
    [0x2112]="External Negative Torque Limit Value"
    [0x2113]="Emergency Stop Torque"
    [0x2114]="P/PI Control Switching Mode"
    [0x2115]="P Control Switching Torque"
    [0x2116]="P Control Switching Speed"
    [0x2117]="P Control Switching Acceleration"
    [0x2118]="P Control Switching Positional Error"
    [0x2119]="Gain Switching Mode"
    [0x211A]="Gain Switching Time 1"
    [0x211B]="Gain Switching Time 2"
    [0x211C]="Gain Switching Waiting Time 1"
    [0x211D]="Gain Swithcing Waiting Time 2"
    [0x211E]="Dead Band For Position Control"
    [0x211F]="Drive Control Input 1"
    [0x2120]="Drive Status Output 1"

    [0x2300]="Jog Operation Speed"
    [0x2301]="Speed Command Acceleration Time"
    [0x2302]="Speed Command Decceleration Time"
    [0x2303]="Speed Command S-curve Time"
    [0x2304]="Programmed Jog Operation Speed 1"
    [0x2305]="Programmed Jog Operation Speed 2"
    [0x2306]="Programmed Jog Operation Speed 3"
    [0x2307]="Programmed Jog Operation Speed 4"
    [0x2308]="Programmed Jog Operation Time 1"
    [0x2309]="Programmed Jog Operation Time 2"
    [0x230A]="Programmed Jog Operation Time 3"
    [0x230B]="Programmed Jog Operation Time 4"
    [0x230C]="Index Pulse Search Speed"
    [0x230D]="Speed Limit Function Setting"
    [0x230E]="Speed Limit Value At Torque Control Mode"
    [0x230F]="Overspeed Detection Level"
    [0x2310]="Excessive Speed Error Detection Level"
    [0x2311]="Servo-Lock Function Setting"

    [0x2400]="Software Position Limit Function Setting"
    [0x2401]="INPOS1 Output Range"
    [0x2402]="INPOS1 Output Time"
    [0x2403]="INPOS2 Output Range"
    [0x2404]="ZSPD Output Range"
    [0x2405]="TGON Output Range"
    [0x2406]="INSPD Output Range"
    [0x2407]="BRAKE Output Speed"
    [0x2408]="BRAKE Output Delay Time"
    [0x2409]="Torque Limit at Homing Using Stopping"
    [0x240A]="Duration Time at Homing Using Stopping"
    [0x240B]="Modulo Mode"
    [0x240C]="Modulo Factor"
    [0x240D]="User Drive Name"
    [0x240E]="Individual Parameter Save"

    [0x2500]="Adaptive Filter Function Setting"
    [0x2501]="Notch Filter 1 Frequency"
    [0x2502]="Notch Filter 1 Width"
    [0x2503]="Notch Filter 1 Depth"
    [0x2504]="Notch Filter "
    [0x2505]="Notch Filter "
    [0x2506]="Notch Filter "
    [0x2507]="Notch Filter "
    [0x2508]="Notch Filter "
    [0x2509]="Notch Filter "
    [0x250A]="Notch Filter "
    [0x250B]="Notch Filter "
    [0x250C]="Notch Filter "
    [0x250D]="Online Gain Tuning Mode"
    [0x250E]="System Rigidity for Gain Tuning"
    [0x250F]="Online Gain Tuning Adaptation Speed"
    [0x2510]="Off-line Gain Tuning Direction"
    [0x2511]="Off-line Gain Tuning Distance"
    [0x2512]="Disturbance Observer Gain"
    [0x2513]="Disturbance Observer Filter Time Constant"
    [0x2514]="Current Controller Gain"
    [0x2515]="Vibration Suppression Filter Configuration"
    [0x2516]="Vibration Suppression Filter 1 Frequency"
    [0x2517]="Vibration Suppression Filter 1 Dumping"
    [0x2518]="Vibration Suppression Filter 2 Frequency"
    [0x2519]="Vibration Suppression Filter 2 Dumping"

    [0x605A]="Quick Stop Option Code"
    [0x605B]="Shutdown Option Code"
    [0x605C]="Disable Operation Option Code"
    [0x605D]="Halt Option Code"
    [0x6072]="Maximum Torque"
    [0x6076]="Motor Rated Torque"
    [0x607D]="Software Position Limit"
    [0x6085]="Quick Stop Deceleration"
    [0x6087]="Torque Slope"
    [0x6091]="Gear Ratio"
    [0x6098]="Homing Method"
    [0x60B0]="Position Offset"
    [0x60B1]="Velocity Offset"
    [0x60B2]="Torque Offset"
    [0x60B8]="Touch Probe Function"
    [0x60B9]="Touch Probe Status"
    [0x60E0]="Positive Torque Limit Value"
    [0x60E1]="Negative Torque Limit Value"
    [0x60FD]="Digital Inputs"
    [0x60FE]="Digital Outputs"
)

check_sanity() {
    [[ ! -f $ETHERCAT ]] && echo "ETHERCAT tool path is invalid: $ETHERCAT"
    for reg in ${REGS[*]}; do
        [[ ! -n ${TYPES["$reg"] + 1} ]] && echo "TYPES has no $reg key"
        [[ ! -n ${NAMES["$reg"] + 1} ]] && echo "NAMES has no $reg key"
    done
}

check_result=$(check_sanity)
if [[ ! -z $check_result  ]]; then
    echo "Result:"
    echo "$check_result"
    echo "There were errors... Bailout"
    exit 1
fi

read_reg() {
    local drive=$1
    local reg=$2
    local ind=$3
    local val=$($ETHERCAT -m 0 -p $drive upload -t ${TYPES["$reg"]} $reg $ind)
    echo $val
}

print_header() {
    local w=$1
    local header="\n %-${w}s %-${w}s %-${w}s %-${w}s\n"
    echo "Time: $(date)"
    printf "$header" "REG" "AZIM" "ELEV" "NAME" 
    local delim="=================================================="
    delim=$delim$delim"\n"
    printf $delim
}

print_register() {
    local w=$1
    local reg="$2"
    local val0="$3"
    local val1="$4"
    local name="$5"
    local format=" %-${w}s %-${w}s %-${w}s %-${w}s\n"
    printf "$format" "$reg" "$val0" "$val1" "$name"
}

read_all() {
    local w=24
    print_header $w
    for reg in ${REGS[*]}; do
        local name="${NAMES["$reg"]}"
        if [[ ! -n ${INDICES["$reg"] + 1} ]]; then
            local val0=$(read_reg 0 $reg 0)
            local val1=$(read_reg 1 $reg 0)
            print_register $w $reg "$val0" "$val1" "$name"
        else
            local indices=${INDICES["$reg"]}
            for i in $(seq 1 $indices); do
                local val0=$(read_reg 0 $reg $i)
                local val1=$(read_reg 1 $reg $i)                
                print_register $w "$reg:$i" "$val0" "$val1" "$name"
            done
        fi
    done
}

read_all
