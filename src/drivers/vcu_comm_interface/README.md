# VCU Communication Interface

## About this package

This package implements the interface which is used for communication between the computer box and the VCU. In previous years this has been done using the CAN interface (look at `bernina_can_interface` or `pilatus_can_interface` in `autonomous_2023` if you are interested). Starting from the 2023 season however, we use Ethernet instead. This allows us for:

* faster communication,
* reducing the amount of messages we are sending around by implementing bigger messages (messages in CAN can have max. 8 bytes vs. 65,507 bytes for UDP),
* most importantly - we can now use the PTP protocol to synchronize time in VCU and CB, which gives us same timestamps in VCU logs and CB ROSBags (easier debugging).

For communication via Ethernet we use User Datagram Protocol (UDP). The full list of UDP messages which are being sent between CB and VCU can be found [here](https://docs.google.com/spreadsheets/d/1n4my9LxAsZ7itZo5gaFWMo-OxxthxuZKfkwjsQU4OBA/edit#gid=1854279138).

## Implementation

For the implementation of the interface we use POSIX libraries (have a look at `socket.h`). It consists of three important classes:

1. `Socket` - represents a UDP socket which we can use to either send or receive data between different addresses.
2. `Sender` - handles the process of sending data *to the VCU from the CB*. `Sender` listens to specific ROS messages which it then processes & transforms into data buffers and sends to VCU using UDP via previously initialized socket.
3. `Receiver` - handles the process of receiving data from VCU. `Receiver` uses a socket to listen for UDP messages, which it then transforms to ROS messages. These are then published to specific topics.


Maintainer: Stanislaw Piasecki
