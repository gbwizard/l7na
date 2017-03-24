#!/bin/bash

DIR=/home/dmrl-3/DMRL_3/dev/GlobalRep/dmrl_3/src/antenna/ethercat-1.5.2/tool

show_label() {
    echo -e "\nRead: $@"
}

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


#show_label "0:0x1000, Device Type"
#$DIR/ethercat -m 0 upload -p 0 --type uint32 0x1000 0

#show_label "0:0x1000, Device Type"
#$DIR/ethercat -m 0 upload -p 1 --type uint32 0x1000 0

if false; then
show_label "1:0x1001, Error Register"
$DIR/ethercat -m 0 upload -p 1 --type int8 0x1001 0

show_label "0:0x2000, Motor ID"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2000 0

show_label "1:0x2000, Motor ID"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2000 0

show_label "0:0x2001, Encoder Type"
$DIR/ethercat -m 0 upload -p 0 --type uint16 0x2001 0

show_label "1:0x2001, Encoder Type"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2001 0

show_label "0:0x2002, Encoder Resolution"
$DIR/ethercat -m 0 upload -p 0 --type uint16 0x2002 0

show_label "1:0x2002, Encoder Resolution"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2002 0

show_label "0x2003, Power Fail Mode"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2003 0

show_label "0:0x2022, External Encoder Type"
$DIR/ethercat -m 0 upload -p 0 --type uint16 0x2022 0

show_label "1:0x2022, External Encoder Type"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2022 0


#Absolute Encoder Reset [0x2702]
# multi-turn data (0x260F)
# single-turn data (0x260D)
# actual position value (0x6064)
# home offset (0x607C)
# actual position value (0x6064)

show_label "0: 0x260F, The Multi-turn Data Display [INT]"
$DIR/ethercat -m 0 upload -p 0 --type int16 0x260F 0

show_label "1: 0x260F, The Multi-turn Data Display [INT]"
$DIR/ethercat -m 0 upload -p 1 --type int16 0x260F 0

show_label "0: 0x2610, The Room Temperature Display [UINT]"
$DIR/ethercat -m 0 upload -p 0 --type uint16 0x2610 0

show_label "1: 0x2610, The Room Temperature Display [UINT]"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x2610 0


show_label "0: 0x260D, The Single-turn Data (Pulse) Display (Single-turn Data) [DINT]"
$DIR/ethercat -m 0 upload -p 0 --type int32 0x260D 0

show_label "1: 0x260D, The Single-turn Data (Pulse) Display (Single-turn Data) [DINT]"
$DIR/ethercat -m 0 upload -p 1 --type int32 0x260D 0

show_label "0: 0x6041, Statusword [UINT]"
$DIR/ethercat -m 0 upload -p 0 --type uint16 0x6041 0

show_label "1: 0x6041, Statusword [UINT]"
$DIR/ethercat -m 0 upload -p 1 --type uint16 0x6041 0

show_label "0: 0x6063, The Position Actual Internal Value [DINT]"
$DIR/ethercat -m 0 upload -p 0 --type int32 0x6063 0

show_label "1: 0x6063, The Position Actual Internal Value [DINT]"
$DIR/ethercat -m 0 upload -p 1 --type int32 0x6063 0

show_label "0: 0x6064, The Position Actual Value [DINT]"
$DIR/ethercat -m 0 upload -p 0 --type int32 0x6064 0

show_label "1: 0x6064, The Position Actual Value [DINT]"
$DIR/ethercat -m 0 upload -p 1 --type int32 0x6064 0


show_label "0: 0x607C, The Home Offset [DINT]"
$DIR/ethercat -m 0 upload -p 0 --type int32 0x607C 0

show_label "1: 0x607C, The Home Offset [DINT]"
$DIR/ethercat -m 0 upload -p 1 --type int32 0x607C 0


show_label "0: 0x60FC, The Position Demand Internal Value [DINT]"
$DIR/ethercat -m 0 upload -p 0 --type int32 0x60FC 0

show_label "1: 0x60FC, The Position Demand Internal Value [DINT]"
$DIR/ethercat -m 0 upload -p 1 --type int32 0x60FC 0
fi

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
        value=$($DIR/ethercat -m 0 upload -p $i --type $app_data_type $address $subindex)
        show_label "$i: $address:$subindex, $description [$app_data_type] = $value"
    done
}

#read_reg INT 0x200E 0 "Position Scale Numerator"
#read_reg INT 0x200F 0 "Position Scale Denominator"
read_reg INT  0x6040 0 "Controlword"
read_reg INT  0x6041 0 "Statusword"
read_reg INT  0x605A 0 "Quick Stop Option"
read_reg SINT 0x6060 0 "Operation Mode"
read_reg DINT 0x607A 0 "Target Position"
read_reg DINT 0x6062 0 "Demand Position"
read_reg DINT 0x6064 0 "Actual Position"
read_reg DINT 0x6081 0 "Profile velocity"
