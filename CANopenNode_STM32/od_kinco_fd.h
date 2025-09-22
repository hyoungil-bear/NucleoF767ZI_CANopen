// Auto-generated data-only header from EDS: Kinco_FD_20250417_V1.2.eds
// Generated on 2025-09-19T04:12:59
#pragma once
#include "od_common.h"

// Device Info
#define KINCO_FD_DEVICE_VENDORNAME "Kinco Electric (Shenzhen) Ltd."
#define KINCO_FD_DEVICE_VENDORNUMBER 0x00000300
#define KINCO_FD_DEVICE_PRODUCTNAME "FD driver"
#define KINCO_FD_DEVICE_PRODUCTNUMBER 0x4644
#define KINCO_FD_DEVICE_REVISIONNUMBER "0"
#define KINCO_FD_DEVICE_ORDERCODE "FD driver"

// Index macros
#define KINCO_FD_INDEX_1000 0x1000
#define KINCO_FD_NAME_1000 "Device Type"
#define KINCO_FD_INDEX_1001 0x1001
#define KINCO_FD_NAME_1001 "Error Register"
#define KINCO_FD_INDEX_1005 0x1005
#define KINCO_FD_NAME_1005 "COB ID SYNC"
#define KINCO_FD_INDEX_1006 0x1006
#define KINCO_FD_NAME_1006 "Communication Cycle Period"
#define KINCO_FD_INDEX_1008 0x1008
#define KINCO_FD_NAME_1008 "Manufacturer Device Name"
#define KINCO_FD_INDEX_1009 0x1009
#define KINCO_FD_NAME_1009 "Manufacturer Hardware Version"
#define KINCO_FD_INDEX_100A 0x100A
#define KINCO_FD_NAME_100A "Manufacturer Software Version"
#define KINCO_FD_INDEX_100B 0x100B
#define KINCO_FD_NAME_100B "Device ID"
#define KINCO_FD_INDEX_100C 0x100C
#define KINCO_FD_NAME_100C "Guard Time"
#define KINCO_FD_INDEX_100D 0x100D
#define KINCO_FD_NAME_100D "Life Time Factor"
#define KINCO_FD_INDEX_100E 0x100E
#define KINCO_FD_NAME_100E "Node Guarding ID"
#define KINCO_FD_INDEX_1010 0x1010
#define KINCO_FD_NAME_1010 "Group_Store"
#define KINCO_FD_1010_SUBNUMBER 3
#define KINCO_FD_INDEX_1011 0x1011
#define KINCO_FD_NAME_1011 "Restore Default Parameters"
#define KINCO_FD_1011_SUBNUMBER 2
#define KINCO_FD_INDEX_1014 0x1014
#define KINCO_FD_NAME_1014 "COB ID EMCY"
#define KINCO_FD_INDEX_1016 0x1016
#define KINCO_FD_NAME_1016 "Consumer Heartbeat Time"
#define KINCO_FD_1016_SUBNUMBER 2
#define KINCO_FD_INDEX_1017 0x1017
#define KINCO_FD_NAME_1017 "Producer Heartbeat Time"
#define KINCO_FD_INDEX_1018 0x1018
#define KINCO_FD_NAME_1018 "Identity Object"
#define KINCO_FD_1018_SUBNUMBER 5
#define KINCO_FD_INDEX_1400 0x1400
#define KINCO_FD_NAME_1400 "Receive PDO Communication Parameter 0"
#define KINCO_FD_1400_SUBNUMBER 4
#define KINCO_FD_INDEX_1401 0x1401
#define KINCO_FD_NAME_1401 "Receive PDO Communication Parameter 1"
#define KINCO_FD_1401_SUBNUMBER 4
#define KINCO_FD_INDEX_1402 0x1402
#define KINCO_FD_NAME_1402 "Receive PDO Communication Parameter 2"
#define KINCO_FD_1402_SUBNUMBER 4
#define KINCO_FD_INDEX_1403 0x1403
#define KINCO_FD_NAME_1403 "Receive PDO Communication Parameter 3"
#define KINCO_FD_1403_SUBNUMBER 4
#define KINCO_FD_INDEX_1404 0x1404
#define KINCO_FD_NAME_1404 "Receive PDO Communication Parameter 4"
#define KINCO_FD_1404_SUBNUMBER 4
#define KINCO_FD_INDEX_1405 0x1405
#define KINCO_FD_NAME_1405 "Receive PDO Communication Parameter 5"
#define KINCO_FD_1405_SUBNUMBER 4
#define KINCO_FD_INDEX_1406 0x1406
#define KINCO_FD_NAME_1406 "Receive PDO Communication Parameter 6"
#define KINCO_FD_1406_SUBNUMBER 4
#define KINCO_FD_INDEX_1407 0x1407
#define KINCO_FD_NAME_1407 "Receive PDO Communication Parameter 7"
#define KINCO_FD_1407_SUBNUMBER 4
#define KINCO_FD_INDEX_1600 0x1600
#define KINCO_FD_NAME_1600 "Receive PDO Mapping Parameter 0"
#define KINCO_FD_1600_SUBNUMBER 9
#define KINCO_FD_INDEX_1601 0x1601
#define KINCO_FD_NAME_1601 "Receive PDO Mapping Parameter 1"
#define KINCO_FD_1601_SUBNUMBER 9
#define KINCO_FD_INDEX_1602 0x1602
#define KINCO_FD_NAME_1602 "Receive PDO Mapping Parameter 2"
#define KINCO_FD_1602_SUBNUMBER 9
#define KINCO_FD_INDEX_1603 0x1603
#define KINCO_FD_NAME_1603 "Receive PDO Mapping Parameter 3"
#define KINCO_FD_1603_SUBNUMBER 9
#define KINCO_FD_INDEX_1604 0x1604
#define KINCO_FD_NAME_1604 "Receive PDO Mapping Parameter 4"
#define KINCO_FD_1604_SUBNUMBER 9
#define KINCO_FD_INDEX_1605 0x1605
#define KINCO_FD_NAME_1605 "Receive PDO Mapping Parameter 5"
#define KINCO_FD_1605_SUBNUMBER 9
#define KINCO_FD_INDEX_1606 0x1606
#define KINCO_FD_NAME_1606 "Receive PDO Mapping Parameter 6"
#define KINCO_FD_1606_SUBNUMBER 9
#define KINCO_FD_INDEX_1607 0x1607
#define KINCO_FD_NAME_1607 "Receive PDO Mapping Parameter 7"
#define KINCO_FD_1607_SUBNUMBER 9
#define KINCO_FD_INDEX_1800 0x1800
#define KINCO_FD_NAME_1800 "Transmit PDO Communication Parameter 0"
#define KINCO_FD_1800_SUBNUMBER 6
#define KINCO_FD_INDEX_1801 0x1801
#define KINCO_FD_NAME_1801 "Transmit PDO Communication Parameter 1"
#define KINCO_FD_1801_SUBNUMBER 6
#define KINCO_FD_INDEX_1802 0x1802
#define KINCO_FD_NAME_1802 "Transmit PDO Communication Parameter 2"
#define KINCO_FD_1802_SUBNUMBER 6
#define KINCO_FD_INDEX_1803 0x1803
#define KINCO_FD_NAME_1803 "Transmit PDO Communication Parameter 3"
#define KINCO_FD_1803_SUBNUMBER 6
#define KINCO_FD_INDEX_1804 0x1804
#define KINCO_FD_NAME_1804 "Transmit PDO Communication Parameter 4"
#define KINCO_FD_1804_SUBNUMBER 6
#define KINCO_FD_INDEX_1805 0x1805
#define KINCO_FD_NAME_1805 "Transmit PDO Communication Parameter 5"
#define KINCO_FD_1805_SUBNUMBER 6
#define KINCO_FD_INDEX_1806 0x1806
#define KINCO_FD_NAME_1806 "Transmit PDO Communication Parameter 6"
#define KINCO_FD_1806_SUBNUMBER 6
#define KINCO_FD_INDEX_1807 0x1807
#define KINCO_FD_NAME_1807 "Transmit PDO Communication Parameter 7"
#define KINCO_FD_1807_SUBNUMBER 6
#define KINCO_FD_INDEX_1A00 0x1A00
#define KINCO_FD_NAME_1A00 "Transmit PDO Mapping Parameter 0"
#define KINCO_FD_1A00_SUBNUMBER 9
#define KINCO_FD_INDEX_1A01 0x1A01
#define KINCO_FD_NAME_1A01 "Transmit PDO Mapping Parameter 1"
#define KINCO_FD_1A01_SUBNUMBER 9
#define KINCO_FD_INDEX_1A02 0x1A02
#define KINCO_FD_NAME_1A02 "Transmit PDO Mapping Parameter 2"
#define KINCO_FD_1A02_SUBNUMBER 9
#define KINCO_FD_INDEX_1A03 0x1A03
#define KINCO_FD_NAME_1A03 "Transmit PDO Mapping Parameter 3"
#define KINCO_FD_1A03_SUBNUMBER 9
#define KINCO_FD_INDEX_1A04 0x1A04
#define KINCO_FD_NAME_1A04 "Transmit PDO Mapping Parameter 4"
#define KINCO_FD_1A04_SUBNUMBER 9
#define KINCO_FD_INDEX_1A05 0x1A05
#define KINCO_FD_NAME_1A05 "Transmit PDO Mapping Parameter 5"
#define KINCO_FD_1A05_SUBNUMBER 9
#define KINCO_FD_INDEX_1A06 0x1A06
#define KINCO_FD_NAME_1A06 "Transmit PDO Mapping Parameter 6"
#define KINCO_FD_1A06_SUBNUMBER 9
#define KINCO_FD_INDEX_1A07 0x1A07
#define KINCO_FD_NAME_1A07 "Transmit PDO Mapping Parameter 7"
#define KINCO_FD_1A07_SUBNUMBER 9
#define KINCO_FD_INDEX_2010 0x2010
#define KINCO_FD_NAME_2010 "Digital In and Output"
#define KINCO_FD_2010_SUBNUMBER 26
#define KINCO_FD_INDEX_2501 0x2501
#define KINCO_FD_NAME_2501 "ADC Real Data"
#define KINCO_FD_2501_SUBNUMBER 11
#define KINCO_FD_INDEX_2502 0x2502
#define KINCO_FD_NAME_2502 "Analog Input Config"
#define KINCO_FD_2502_SUBNUMBER 17
#define KINCO_FD_INDEX_2508 0x2508
#define KINCO_FD_NAME_2508 "Pulse Control"
#define KINCO_FD_2508_SUBNUMBER 14
#define KINCO_FD_INDEX_2601 0x2601
#define KINCO_FD_NAME_2601 "Error Status"
#define KINCO_FD_INDEX_2602 0x2602
#define KINCO_FD_NAME_2602 "Error Status2"
#define KINCO_FD_INDEX_2680 0x2680
#define KINCO_FD_NAME_2680 "Warning_Word"
#define KINCO_FD_INDEX_2F81 0x2F81
#define KINCO_FD_NAME_2F81 "ECAN Baudrate"
#define KINCO_FD_INDEX_2FE0 0x2FE0
#define KINCO_FD_NAME_2FE0 "RS232 Bandrate"
#define KINCO_FD_INDEX_2FE2 0x2FE2
#define KINCO_FD_NAME_2FE2 "RS485 Bandrate"
#define KINCO_FD_INDEX_2FF0 0x2FF0
#define KINCO_FD_NAME_2FF0 "Group_Panel"
#define KINCO_FD_2FF0_SUBNUMBER 4
#define KINCO_FD_INDEX_3011 0x3011
#define KINCO_FD_NAME_3011 "ECAN Parameters Setting"
#define KINCO_FD_3011_SUBNUMBER 5
#define KINCO_FD_INDEX_6004 0x6004
#define KINCO_FD_NAME_6004 "absolute position of motor"
#define KINCO_FD_INDEX_6007 0x6007
#define KINCO_FD_NAME_6007 "Abort_Connection_Mode"
#define KINCO_FD_INDEX_603F 0x603F
#define KINCO_FD_NAME_603F "error code for DS301"
#define KINCO_FD_INDEX_6040 0x6040
#define KINCO_FD_NAME_6040 "Controlword"
#define KINCO_FD_INDEX_6041 0x6041
#define KINCO_FD_NAME_6041 "Statusword"
#define KINCO_FD_INDEX_605A 0x605A
#define KINCO_FD_NAME_605A "Quick_stop_option_code"
#define KINCO_FD_INDEX_605B 0x605B
#define KINCO_FD_NAME_605B "Shutdown_Stop_Mode"
#define KINCO_FD_INDEX_605C 0x605C
#define KINCO_FD_NAME_605C "Disable_Stop_Mode"
#define KINCO_FD_INDEX_605D 0x605D
#define KINCO_FD_NAME_605D "Halt_Mode"
#define KINCO_FD_INDEX_605E 0x605E
#define KINCO_FD_NAME_605E "Fault_reaction_option_code"
#define KINCO_FD_INDEX_6060 0x6060
#define KINCO_FD_NAME_6060 "Modes_of_operation"
#define KINCO_FD_INDEX_6061 0x6061
#define KINCO_FD_NAME_6061 "Operation_Mode_Buff"
#define KINCO_FD_INDEX_6063 0x6063
#define KINCO_FD_NAME_6063 "Position_actual_value_"
#define KINCO_FD_INDEX_6064 0x6064
#define KINCO_FD_NAME_6064 "Position_actual_value"
#define KINCO_FD_INDEX_6065 0x6065
#define KINCO_FD_NAME_6065 "Max_Following_Error"
#define KINCO_FD_INDEX_6067 0x6067
#define KINCO_FD_NAME_6067 "Target_Pos_Window"
#define KINCO_FD_INDEX_6068 0x6068
#define KINCO_FD_NAME_6068 "Position_Window_time"
#define KINCO_FD_INDEX_606B 0x606B
#define KINCO_FD_NAME_606B "Velocity_demand_value"
#define KINCO_FD_INDEX_606C 0x606C
#define KINCO_FD_NAME_606C "speed_real"
#define KINCO_FD_INDEX_6071 0x6071
#define KINCO_FD_NAME_6071 "Target_torque"
#define KINCO_FD_INDEX_6072 0x6072
#define KINCO_FD_NAME_6072 "Max_Torque%"
#define KINCO_FD_INDEX_6073 0x6073
#define KINCO_FD_NAME_6073 "Max_current"
#define KINCO_FD_INDEX_6075 0x6075
#define KINCO_FD_NAME_6075 "Rated_Current"
#define KINCO_FD_INDEX_6076 0x6076
#define KINCO_FD_NAME_6076 "Rated_Torque"
#define KINCO_FD_INDEX_6077 0x6077
#define KINCO_FD_NAME_6077 "Actual_Torque"
#define KINCO_FD_INDEX_6078 0x6078
#define KINCO_FD_NAME_6078 "I_q"
#define KINCO_FD_INDEX_6079 0x6079
#define KINCO_FD_NAME_6079 "Real_DCBUS_mV"
#define KINCO_FD_INDEX_607A 0x607A
#define KINCO_FD_NAME_607A "Target_position"
#define KINCO_FD_INDEX_607C 0x607C
#define KINCO_FD_NAME_607C "Home_offset"
#define KINCO_FD_INDEX_607D 0x607D
#define KINCO_FD_NAME_607D "Group_Soft_Limit"
#define KINCO_FD_607D_SUBNUMBER 3
#define KINCO_FD_INDEX_607E 0x607E
#define KINCO_FD_NAME_607E "Polarity"
#define KINCO_FD_INDEX_607F 0x607F
#define KINCO_FD_NAME_607F "Max_Speed"
#define KINCO_FD_INDEX_6080 0x6080
#define KINCO_FD_NAME_6080 "Max_Speed_RPM"
#define KINCO_FD_INDEX_6081 0x6081
#define KINCO_FD_NAME_6081 "Profile_velocity"
#define KINCO_FD_INDEX_6083 0x6083
#define KINCO_FD_NAME_6083 "Profile_acceleration"
#define KINCO_FD_INDEX_6084 0x6084
#define KINCO_FD_NAME_6084 "Profile_deceleration"
#define KINCO_FD_INDEX_6085 0x6085
#define KINCO_FD_NAME_6085 "Quick_stop_deceleration"
#define KINCO_FD_INDEX_6098 0x6098
#define KINCO_FD_NAME_6098 "Homing_method"
#define KINCO_FD_INDEX_6099 0x6099
#define KINCO_FD_NAME_6099 "Group_Homing_Speed"
#define KINCO_FD_6099_SUBNUMBER 7
#define KINCO_FD_INDEX_609A 0x609A
#define KINCO_FD_NAME_609A "Homing_Accelaration"
#define KINCO_FD_INDEX_60C0 0x60C0
#define KINCO_FD_NAME_60C0 "Interpolation_sub_mode_select"
#define KINCO_FD_INDEX_60C1 0x60C1
#define KINCO_FD_NAME_60C1 "Interpolation_data_record"
#define KINCO_FD_60C1_SUBNUMBER 3
#define KINCO_FD_INDEX_60C2 0x60C2
#define KINCO_FD_NAME_60C2 "Interpolation_time_period"
#define KINCO_FD_60C2_SUBNUMBER 3
#define KINCO_FD_INDEX_60C3 0x60C3
#define KINCO_FD_NAME_60C3 "Interpolation_sync_definition"
#define KINCO_FD_60C3_SUBNUMBER 3
#define KINCO_FD_INDEX_60C4 0x60C4
#define KINCO_FD_NAME_60C4 "Interpolation_data_configuration"
#define KINCO_FD_60C4_SUBNUMBER 7
#define KINCO_FD_INDEX_60F4 0x60F4
#define KINCO_FD_NAME_60F4 "Following_error_actual_value"
#define KINCO_FD_INDEX_60F6 0x60F6
#define KINCO_FD_NAME_60F6 "Group_Current_Loop"
#define KINCO_FD_60F6_SUBNUMBER 14
#define KINCO_FD_INDEX_60F9 0x60F9
#define KINCO_FD_NAME_60F9 "Group_Speed_Loop"
#define KINCO_FD_60F9_SUBNUMBER 11
#define KINCO_FD_INDEX_60FB 0x60FB
#define KINCO_FD_NAME_60FB "Group_Position_Loop"
#define KINCO_FD_60FB_SUBNUMBER 6
#define KINCO_FD_INDEX_60FC 0x60FC
#define KINCO_FD_NAME_60FC "Position_demand_value"
#define KINCO_FD_INDEX_60FD 0x60FD
#define KINCO_FD_NAME_60FD "Digital_inputs"
#define KINCO_FD_INDEX_60FF 0x60FF
#define KINCO_FD_NAME_60FF "Target_velocity"
#define KINCO_FD_INDEX_6410 0x6410
#define KINCO_FD_NAME_6410 "Motor_data"
#define KINCO_FD_6410_SUBNUMBER 23
#define KINCO_FD_INDEX_6504 0x6504
#define KINCO_FD_NAME_6504 "Drive_manufacturer"

static const OD_Entry KINCO_FD_TABLE[] = {
    { 0x1000, 0, ODT_UNSIGNED32, OD_ACCESS_RO, "Device Type" },
    { 0x1001, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Error Register" },
    { 0x1005, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID SYNC" },
    { 0x1006, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Communication Cycle Period" },
    { 0x1008, 0, ODT_VISIBLE_STRING, OD_ACCESS_RO, "Manufacturer Device Name" },
    { 0x1009, 0, ODT_VISIBLE_STRING, OD_ACCESS_RO, "Manufacturer Hardware Version" },
    { 0x100A, 0, ODT_VISIBLE_STRING, OD_ACCESS_RO, "Manufacturer Software Version" },
    { 0x100B, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Device ID" },
    { 0x100C, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Guard Time" },
    { 0x100D, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Life Time Factor" },
    { 0x100E, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Node Guarding ID" },
    { 0x1010, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "number of elements" },
    { 0x1010, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "Store_Loop_Data_301" },
    { 0x1010, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "Store_Loop_Data2" },
    { 0x1011, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1011, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "Restore all Default Parameters 1" },
    { 0x1014, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID EMCY" },
    { 0x1016, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1016, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "Consumer Heartbeat Time" },
    { 0x1017, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Producer Heartbeat Time" },
    { 0x1018, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1018, 1, ODT_UNSIGNED32, OD_ACCESS_RO, "Vendor Id 1" },
    { 0x1018, 2, ODT_UNSIGNED32, OD_ACCESS_RO, "Product Code" },
    { 0x1018, 3, ODT_UNSIGNED32, OD_ACCESS_RO, "Revision number" },
    { 0x1018, 4, ODT_UNSIGNED32, OD_ACCESS_RO, "Serial number" },
    { 0x1400, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1400, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1400, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1400, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1401, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1401, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1401, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1401, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1402, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1402, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1402, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1402, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1403, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1403, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1403, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1403, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1404, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1404, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1404, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1404, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1405, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1405, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1405, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1405, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1406, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1406, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1406, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1406, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1407, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1407, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1407, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1407, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1600, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1600, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1600, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1600, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1600, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1600, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1600, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1600, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1600, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1601, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1601, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1601, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1601, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1601, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1601, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1601, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1601, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1601, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1602, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1602, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1602, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1602, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1602, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1602, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1602, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1602, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1602, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1603, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1603, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1603, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1603, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1603, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1603, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1603, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1603, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1603, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1604, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1604, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1604, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1604, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1604, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1604, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1604, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1604, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1604, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1605, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1605, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1605, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1605, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1605, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1605, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1605, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1605, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1605, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1606, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1606, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1606, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1606, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1606, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1606, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1606, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1606, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1606, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1607, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1607, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1607, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1607, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1607, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1607, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1607, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1607, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1607, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1800, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1800, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1800, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1800, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1800, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180004" },
    { 0x1800, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1801, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1801, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1801, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1801, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1801, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180104" },
    { 0x1801, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1802, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1802, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1802, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1802, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1802, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180204" },
    { 0x1802, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1803, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1803, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1803, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1803, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1803, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180304" },
    { 0x1803, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1804, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1804, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1804, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1804, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1804, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180404" },
    { 0x1804, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1805, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1805, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1805, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1805, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1805, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180504" },
    { 0x1805, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1806, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1806, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1806, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1806, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1806, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Compatibility Entry" },
    { 0x1806, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1807, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x1807, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "COB ID 1" },
    { 0x1807, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Transmission Type" },
    { 0x1807, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Inhibit Time" },
    { 0x1807, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Reserved_180704" },
    { 0x1807, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Event Timer" },
    { 0x1A00, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A00, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A00, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A00, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A00, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A00, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A00, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A00, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A00, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A01, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A01, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A01, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A01, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A01, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A01, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A01, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A01, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A01, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A02, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A02, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A02, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A02, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A02, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A02, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A02, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A02, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A02, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A03, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A03, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A03, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A03, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A03, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A03, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A03, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A03, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A03, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A04, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A04, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A04, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A04, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A04, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A04, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A04, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A04, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A04, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A05, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A05, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A05, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A05, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A05, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A05, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A05, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A05, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A05, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A06, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A06, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A06, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A06, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A06, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A06, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A06, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A06, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A06, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x1A07, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Number of entries" },
    { 0x1A07, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 1" },
    { 0x1A07, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 2" },
    { 0x1A07, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 3" },
    { 0x1A07, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 4" },
    { 0x1A07, 5, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 5" },
    { 0x1A07, 6, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 6" },
    { 0x1A07, 7, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 7" },
    { 0x1A07, 8, ODT_UNSIGNED32, OD_ACCESS_RW, "PDO Mapping Entry 8" },
    { 0x2010, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x2010, 1, ODT_UNSIGNED16, OD_ACCESS_RW, "Input Polarity" },
    { 0x2010, 2, ODT_UNSIGNED16, OD_ACCESS_RW, "Input Simulation" },
    { 0x2010, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Din1 Function" },
    { 0x2010, 4, ODT_UNSIGNED16, OD_ACCESS_RW, "Din2 Function" },
    { 0x2010, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Din3 Function" },
    { 0x2010, 6, ODT_UNSIGNED16, OD_ACCESS_RW, "Din4 Function" },
    { 0x2010, 7, ODT_UNSIGNED16, OD_ACCESS_RW, "Din5 Function" },
    { 0x2010, 8, ODT_UNSIGNED16, OD_ACCESS_RW, "Din6 Function" },
    { 0x2010, 9, ODT_UNSIGNED16, OD_ACCESS_RW, "Din7 Function" },
    { 0x2010, 10, ODT_UNSIGNED16, OD_ACCESS_RW, "Dout2 Function" },
    { 0x2010, 11, ODT_UNSIGNED16, OD_ACCESS_RW, "Dout3 Function" },
    { 0x2010, 12, ODT_UNSIGNED16, OD_ACCESS_RW, "Dout4 Function" },
    { 0x2010, 13, ODT_UNSIGNED16, OD_ACCESS_RW, "Dout5 Function" },
    { 0x2010, 14, ODT_UNSIGNED16, OD_ACCESS_RO, "Output Status" },
    { 0x2010, 15, ODT_UNSIGNED16, OD_ACCESS_RO, "Output Virtual Status" },
    { 0x2010, 16, ODT_UNSIGNED16, OD_ACCESS_RO, "Output Sys Status" },
    { 0x2501, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x2501, 4, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC1 Data[0]" },
    { 0x2501, 5, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC2 Data[0]" },
    { 0x2501, 6, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC1 Data[1]" },
    { 0x2501, 7, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC2 Data[1]" },
    { 0x2501, 8, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC1 Data[2]" },
    { 0x2501, 9, ODT_UNSIGNED16, OD_ACCESS_RO, "ADC2 Data[2]" },
    { 0x2502, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x2502, 1, ODT_UNSIGNED16, OD_ACCESS_RW, "Analog1 Filter" },
    { 0x2502, 2, ODT_INTEGER16, OD_ACCESS_RW, "Analog1 Dead" },
    { 0x2502, 3, ODT_INTEGER16, OD_ACCESS_RW, "Analog1 Offset" },
    { 0x2502, 4, ODT_UNSIGNED16, OD_ACCESS_RW, "Analog2 Filter" },
    { 0x2502, 5, ODT_INTEGER16, OD_ACCESS_RW, "Analog2 Dead" },
    { 0x2502, 6, ODT_INTEGER16, OD_ACCESS_RW, "Analog2 Offset" },
    { 0x2502, 7, ODT_UNSIGNED8, OD_ACCESS_RW, "Analog Speed Control" },
    { 0x2502, 8, ODT_UNSIGNED8, OD_ACCESS_RW, "Analog Torque Control" },
    { 0x2502, 9, ODT_UNSIGNED8, OD_ACCESS_RW, "Analog Max. Torque Control" },
    { 0x2502, 10, ODT_INTEGER16, OD_ACCESS_RO, "Analog2 Value" },
    { 0x2508, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x2508, 1, ODT_INTEGER16, OD_ACCESS_RW, "Gear Factor" },
    { 0x2508, 2, ODT_UNSIGNED16, OD_ACCESS_RW, "Gear Devider" },
    { 0x2508, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "Pulse Mode" },
    { 0x2508, 4, ODT_INTEGER32, OD_ACCESS_RW, "Gear Master Counter" },
    { 0x2508, 5, ODT_INTEGER32, OD_ACCESS_RW, "Gear Slave Counter" },
    { 0x2508, 6, ODT_UNSIGNED16, OD_ACCESS_RW, "Pulse Filter" },
    { 0x2508, 7, ODT_INTEGER16, OD_ACCESS_RW, "Gear Remainder" },
    { 0x2508, 8, ODT_UNSIGNED16, OD_ACCESS_RW, "Pulse Frequency Check" },
    { 0x2508, 9, ODT_UNSIGNED16, OD_ACCESS_RW, "Position Reach Time Window" },
    { 0x2601, 0, ODT_UNSIGNED16, OD_ACCESS_RO, "Error Status" },
    { 0x2602, 0, ODT_UNSIGNED16, OD_ACCESS_RO, "Error Status2" },
    { 0x2680, 0, ODT_UNSIGNED16, OD_ACCESS_RO, "Warning_Word" },
    { 0x2F81, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "ECAN Baudrate" },
    { 0x2FE0, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "RS232 Bandrate" },
    { 0x2FE2, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "RS485 Bandrate" },
    { 0x2FF0, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Panel" },
    { 0x2FF0, 1, ODT_UNSIGNED8, OD_ACCESS_RW, "Store_Data" },
    { 0x2FF0, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Store_Calibrate_Data" },
    { 0x2FF0, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "Store_Motor_Data" },
    { 0x3011, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Number of entries" },
    { 0x3011, 1, ODT_UNSIGNED8, OD_ACCESS_RW, "ECAN Sync Cycle" },
    { 0x3011, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "ECAN Sync Clock" },
    { 0x3011, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "ECAN Flags" },
    { 0x3011, 4, ODT_INTEGER16, OD_ACCESS_RW, "ECAN Sync Data" },
    { 0x6004, 0, ODT_INTEGER32, OD_ACCESS_RO, "absolute position of motor" },
    { 0x6007, 0, ODT_INTEGER16, OD_ACCESS_RW, "Abort_Connection_Mode" },
    { 0x603F, 0, ODT_UNSIGNED16, OD_ACCESS_RO, "error code for DS301" },
    { 0x6040, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Controlword" },
    { 0x6041, 0, ODT_UNSIGNED16, OD_ACCESS_RO, "Statusword" },
    { 0x605A, 0, ODT_INTEGER16, OD_ACCESS_RW, "Quick_stop_option_code" },
    { 0x605B, 0, ODT_INTEGER16, OD_ACCESS_RW, "Shutdown_Stop_Mode" },
    { 0x605C, 0, ODT_INTEGER16, OD_ACCESS_RW, "Disable_Stop_Mode" },
    { 0x605D, 0, ODT_INTEGER16, OD_ACCESS_RW, "Halt_Mode" },
    { 0x605E, 0, ODT_INTEGER16, OD_ACCESS_RW, "Fault_reaction_option_code" },
    { 0x6060, 0, ODT_INTEGER8, OD_ACCESS_RW, "Modes_of_operation" },
    { 0x6061, 0, ODT_INTEGER8, OD_ACCESS_RO, "Operation_Mode_Buff" },
    { 0x6063, 0, ODT_INTEGER32, OD_ACCESS_RO, "Position_actual_value_" },
    { 0x6064, 0, ODT_INTEGER32, OD_ACCESS_RO, "Position_actual_value" },
    { 0x6065, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Max_Following_Error" },
    { 0x6067, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Target_Pos_Window" },
    { 0x6068, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Position_Window_time" },
    { 0x606B, 0, ODT_INTEGER32, OD_ACCESS_RO, "Velocity_demand_value" },
    { 0x606C, 0, ODT_INTEGER32, OD_ACCESS_RO, "speed_real" },
    { 0x6071, 0, ODT_INTEGER16, OD_ACCESS_RW, "Target_torque" },
    { 0x6072, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Max_Torque%" },
    { 0x6073, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Max_current" },
    { 0x6075, 0, ODT_UNSIGNED32, OD_ACCESS_RO, "Rated_Current" },
    { 0x6076, 0, ODT_UNSIGNED32, OD_ACCESS_RO, "Rated_Torque" },
    { 0x6077, 0, ODT_INTEGER16, OD_ACCESS_RO, "Actual_Torque" },
    { 0x6078, 0, ODT_INTEGER16, OD_ACCESS_RO, "I_q" },
    { 0x6079, 0, ODT_UNSIGNED32, OD_ACCESS_RO, "Real_DCBUS_mV" },
    { 0x607A, 0, ODT_INTEGER32, OD_ACCESS_RW, "Target_position" },
    { 0x607C, 0, ODT_INTEGER32, OD_ACCESS_RW, "Home_offset" },
    { 0x607D, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Soft_Limit" },
    { 0x607D, 1, ODT_INTEGER32, OD_ACCESS_RW, "Soft_Positive_Limit" },
    { 0x607D, 2, ODT_INTEGER32, OD_ACCESS_RW, "Soft_Negative_Limit" },
    { 0x607E, 0, ODT_UNSIGNED8, OD_ACCESS_RW, "Polarity" },
    { 0x607F, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Max_Speed" },
    { 0x6080, 0, ODT_UNSIGNED16, OD_ACCESS_RW, "Max_Speed_RPM" },
    { 0x6081, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Profile_velocity" },
    { 0x6083, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Profile_acceleration" },
    { 0x6084, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Profile_deceleration" },
    { 0x6085, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Quick_stop_deceleration" },
    { 0x6098, 0, ODT_INTEGER8, OD_ACCESS_RW, "Homing_method" },
    { 0x6099, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Homing_Speed" },
    { 0x6099, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "Homing_speeds_Speed_during_search_for_switch 1" },
    { 0x6099, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "Homing_speeds_Speed_during_search_for_zero" },
    { 0x6099, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "Homing_Power_On" },
    { 0x6099, 4, ODT_INTEGER16, OD_ACCESS_RW, "Homing_Current" },
    { 0x6099, 5, ODT_UNSIGNED8, OD_ACCESS_RW, "Home_Offset_Mode" },
    { 0x6099, 6, ODT_UNSIGNED8, OD_ACCESS_RW, "Home_N_Blind" },
    { 0x609A, 0, ODT_UNSIGNED32, OD_ACCESS_RW, "Homing_Accelaration" },
    { 0x60C0, 0, ODT_INTEGER16, OD_ACCESS_RW, "Interpolation_sub_mode_select" },
    { 0x60C1, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Interpolation_data_record_number_of_entries" },
    { 0x60C1, 1, ODT_INTEGER32, OD_ACCESS_RW, "Interpolation_data_record_x1 1" },
    { 0x60C1, 2, ODT_INTEGER32, OD_ACCESS_RW, "Interpolation_data_record_x1 2" },
    { 0x60C2, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Interpolation_time_period_number_of_entries" },
    { 0x60C2, 1, ODT_UNSIGNED8, OD_ACCESS_RW, "Interpolation_time_period_Interpolation_time_units 1" },
    { 0x60C2, 2, ODT_INTEGER8, OD_ACCESS_RW, "Interpolation_time_period_Interpolation_time_index" },
    { 0x60C3, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Interpolation_sync_definition_number_of_entries" },
    { 0x60C3, 1, ODT_UNSIGNED8, OD_ACCESS_RW, "Interpolation_sync_definition_Synchronize_on_group 1" },
    { 0x60C3, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Interpolation_sync_definition_ip_sync_every_n_event" },
    { 0x60C4, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Interpolation_data_configuration_number_of_entries" },
    { 0x60C4, 1, ODT_UNSIGNED32, OD_ACCESS_RW, "Interpolation_data_configuration_Maximum_buffer_size 1" },
    { 0x60C4, 2, ODT_UNSIGNED32, OD_ACCESS_RW, "Interpolation_data_configuration_Actual_buffer_size" },
    { 0x60C4, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "Interpolation_data_configuration_Buffer_organization" },
    { 0x60C4, 4, ODT_UNSIGNED16, OD_ACCESS_RW, "Interpolation_data_configuration_Buffer_position" },
    { 0x60C4, 5, ODT_UNSIGNED8, OD_ACCESS_RW, "Interpolation_data_configuration_Size_of_data_record" },
    { 0x60C4, 6, ODT_UNSIGNED8, OD_ACCESS_RW, "Value_Description_Buffer_clear" },
    { 0x60F4, 0, ODT_INTEGER32, OD_ACCESS_RO, "Following_error_actual_value" },
    { 0x60F6, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Current_Loop" },
    { 0x60F6, 1, ODT_UNSIGNED16, OD_ACCESS_RW, "Kcp" },
    { 0x60F6, 2, ODT_UNSIGNED16, OD_ACCESS_RW, "Kci" },
    { 0x60F6, 3, ODT_UNSIGNED16, OD_ACCESS_RW, "Speed_Limit_Factor" },
    { 0x60F6, 4, ODT_UNSIGNED16, OD_ACCESS_RW, "N_Compensation" },
    { 0x60F6, 5, ODT_INTEGER16, OD_ACCESS_RW, "N_bEMF" },
    { 0x60F6, 6, ODT_INTEGER16, OD_ACCESS_RW, "Comm_Shift_UVW" },
    { 0x60F6, 7, ODT_INTEGER16, OD_ACCESS_RW, "Voltage_Angle_Adjust" },
    { 0x60F6, 8, ODT_INTEGER16, OD_ACCESS_RW, "CMD_q" },
    { 0x60F6, 9, ODT_INTEGER16, OD_ACCESS_RW, "CMD_d" },
    { 0x60F9, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Speed_Loop" },
    { 0x60F9, 1, ODT_UNSIGNED16, OD_ACCESS_RW, "Kvp_0" },
    { 0x60F9, 2, ODT_UNSIGNED16, OD_ACCESS_RW, "Kvi_0" },
    { 0x60F9, 3, ODT_UNSIGNED8, OD_ACCESS_RW, "Notch_N" },
    { 0x60F9, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Notch_on" },
    { 0x60F9, 5, ODT_UNSIGNED8, OD_ACCESS_RW, "Speed_Fb_n" },
    { 0x60F9, 6, ODT_UNSIGNED8, OD_ACCESS_RW, "Speed_mode" },
    { 0x60F9, 7, ODT_UNSIGNED16, OD_ACCESS_RW, "Kvi/32" },
    { 0x60F9, 8, ODT_INTEGER32, OD_ACCESS_RW, "Kvi_sum_limit" },
    { 0x60F9, 9, ODT_UNSIGNED8, OD_ACCESS_RW, "PI_switch" },
    { 0x60FB, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Group_Position_Loop" },
    { 0x60FB, 1, ODT_INTEGER16, OD_ACCESS_RW, "kpp_0" },
    { 0x60FB, 2, ODT_INTEGER16, OD_ACCESS_RW, "K_Velocity_FF" },
    { 0x60FB, 3, ODT_INTEGER16, OD_ACCESS_RW, "K_Acc_FF" },
    { 0x60FB, 4, ODT_UNSIGNED8, OD_ACCESS_RW, "Pos_speed_filter" },
    { 0x60FB, 5, ODT_UNSIGNED16, OD_ACCESS_RW, "Pos_Filter_N" },
    { 0x60FC, 0, ODT_INTEGER32, OD_ACCESS_RO, "Position_demand_value" },
    { 0x60FD, 0, ODT_UNSIGNED32, OD_ACCESS_RO, "Digital_inputs" },
    { 0x60FF, 0, ODT_INTEGER32, OD_ACCESS_RW, "Target_velocity" },
    { 0x6410, 0, ODT_UNSIGNED8, OD_ACCESS_RO, "Motor_data_Number_of_entries" },
    { 0x6410, 1, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor Number" },
    { 0x6410, 2, ODT_UNSIGNED8, OD_ACCESS_RW, "Motor Feedback Type" },
    { 0x6410, 3, ODT_UNSIGNED32, OD_ACCESS_RW, "Motor Feedback Resolution" },
    { 0x6410, 4, ODT_UNSIGNED32, OD_ACCESS_RW, "Motor Feedback Period" },
    { 0x6410, 5, ODT_UNSIGNED8, OD_ACCESS_RW, "Motor Poles Pair" },
    { 0x6410, 6, ODT_UNSIGNED8, OD_ACCESS_RW, "Motor Commutation Mode" },
    { 0x6410, 7, ODT_INTEGER16, OD_ACCESS_RW, "Motor commutation current" },
    { 0x6410, 8, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor commutation delay" },
    { 0x6410, 9, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor IIt protection I" },
    { 0x6410, 10, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor Jr" },
    { 0x6410, 11, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor brake duty cycle" },
    { 0x6410, 12, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor brake delay time" },
    { 0x6410, 13, ODT_UNSIGNED8, OD_ACCESS_RW, "Motor turnning direction" },
    { 0x6410, 14, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor reserve" },
    { 0x6410, 15, ODT_UNSIGNED16, OD_ACCESS_RW, "Motor reserve" },
    { 0x6410, 16, ODT_UNSIGNED16, OD_ACCESS_RO, "Motor using" },
    { 0x6504, 0, ODT_VISIBLE_STRING, OD_ACCESS_RO, "Drive_manufacturer" },
};
static const uint32_t KINCO_FD_TABLE_COUNT = 422;


// ===== PDO/SDO Views =====
// RPDO_COMM
static const OD_PDOComm KINCO_FD_RPDO_COMM[] = {
    { 0x1400, 0, 0, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1401, 0, 1, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1402, 0, 2, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1403, 0, 3, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1404, 0, 4, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1405, 0, 5, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1406, 0, 6, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
    { 0x1407, 0, 7, 0x00000000, 0xFE, 0x0000, 0x0000, 0x0000 },
};
static const uint32_t KINCO_FD_RPDO_COMM_COUNT = 8;

// TPDO_COMM
static const OD_PDOComm KINCO_FD_TPDO_COMM[] = {
    { 0x1800, 1, 0, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1801, 1, 1, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1802, 1, 2, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1803, 1, 3, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1804, 1, 4, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1805, 1, 5, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1806, 1, 6, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
    { 0x1807, 1, 7, 0x00000000, 0x00, 0x0000, 0x0000, 0x0000 },
};
static const uint32_t KINCO_FD_TPDO_COMM_COUNT = 8;

// RPDO_MAP
static const OD_PDOMapping KINCO_FD_RPDO_MAP[] = {
    { 0x1600, 0, 0, 3, {
        { 0x607A, 0x00, 0x20 },
        { 0x6060, 0x00, 0x08 },
        { 0x6040, 0x00, 0x10 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1601, 0, 1, 2, {
        { 0x6081, 0x00, 0x20 },
        { 0x60FF, 0x00, 0x20 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1602, 0, 2, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1603, 0, 3, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1604, 0, 4, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1605, 0, 5, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1606, 0, 6, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1607, 0, 7, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
};
static const uint32_t KINCO_FD_RPDO_MAP_COUNT = 8;

// TPDO_MAP
static const OD_PDOMapping KINCO_FD_TPDO_MAP[] = {
    { 0x1A00, 1, 0, 3, {
        { 0x6041, 0x00, 0x10 },
        { 0x6063, 0x00, 0x20 },
        { 0x6061, 0x00, 0x08 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A01, 1, 1, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A02, 1, 2, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A03, 1, 3, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A04, 1, 4, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A05, 1, 5, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A06, 1, 6, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
    { 0x1A07, 1, 7, 0, {
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
        { 0x0000, 0x00, 0x00 },
    } },
};
static const uint32_t KINCO_FD_TPDO_MAP_COUNT = 8;
