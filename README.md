# trelpo
Wireless signalling and turnout control decoder for Rocrail

*********************************************************************************************
  Aim is to develop a turnout and signalling system for model railroad track control.
  Communication is wireless.

  To disconnect the decoder from the controlling software in space and time the MOSQUITTO
  MQTT broker is used.

  Hardware setup is as simple as possible.

  Sofware is modularised.

*********************************************************************************************

  The decoder collects sensor information or activates relais and controls hobby RC servo's.

  Target platform is Wemos D1 mini.

  11 digital input/output pins, all pins have interrupt/pwm/I2C/one-wire supported(except D0)
  1 analog input(3.2V max input)
  a Micro USB connection
  compatible with Arduino IDE.
  pins are active LOW.

  All of the IO pins run at 3.3V:
  For sensors all pins are high in No Detection (ND). Each sensor connects a pin to GND at detection(D).
  No consequence for IO voltage.

  For a relay a pin is followed by a transistor/IC. A relay is driven with 5V.

  In stead of having one module for all applications the design aims for different PCB's specific to task
  were the Wemos has to be slotted into.

  8 pins are used.

  trelpo shield                 Function                                          Hardware ID
  trelpo signalling shield      Detection 8 sensors                                   0 0
  trelpo turnout shield         Controlling 4 turnouts                                0 1

  GPIO usage:

  Hardware ID      pins        function
  0 0            D0 .. D7    read contact
  0 1            D0 .. D3    switch relay
  0 1            D4 .. D7    PWM signal for servo

  ROCNET PROTOCOL

  packet payload content:
  byte 1  : groupId
  byte 2  : receiveIdH
  Byte 3  : receiveIdL
  byte 4  : sendIdH
  byte 5  : sendIdL
  byte 6  : group
  byte 7  : code
  byte 8  : length
  byte 9  : data1
  byte 10 : data2
  byte 11 : data3
  byte 12 : data4

  --byte 1 only used for large network. Normally 0.

  --byte 2 only used when more than 255 decoders. Normally 0.

  --byte 3 Rocrail Server default Id is 1

  Broadcast Id = 0
  Decoder   Id = 2 ... 255   Not used for switching decoder

  --byte 4 only used when more than 255 decoders. Normally 0.

  --byte 5 Rocrail Server default Id is 1

  Decoder Id     = 2 ... 255

  --byte 6

  groups
  code   description           remark                              MQTT topic
  0      Host                  Rocrail                             rocnet/ht
  3      Stationary decoders   Multiport for inputs and outputs    rocnet/dc
  7      Clock Fast            Clock                               rocnet/ck
  8      Sensor                Position determination              rocnet/sr
  9      Output                                                    rocnet/ot


  --byte 7

  Code:  bit 7 = 0
       bit 6 and bit 5 represent Type of code
       bit 4 .. 0:  5 bit command 0 .. 31

  Type: 0 - test
        1 - request
        2 - event
        3 - reply

  Sensor

  Actions
  code description data 1  data 2  data 3  data 4  data n
  1     report    addrH¹  addrL¹  status  port    identifier (RFID)

  ¹) Address of the reporting loco.
  The sensor ID is set in the header; Sender.
  A sensor secure ack is done with the stationary acknowledge msg:
  Code = 1
  Port = port

  Output

  Type     Value
  switch     0
  light      1
  servo      2

  Actions
  code description data 1  data 2  data 3
  0       off     type    value   address
  1       on      type    value   address

  --byte 8 Netto number of following data bytes.

  At a speed of 200 KmH a loc runs 64 mm per second in scale H0 and 35 mm per second in scale N.
  To come to a reliable detection reaction a point sensor must be scanned at least 20 times per second in H0
  and at least 10 times per second in scale N.

  For debouncing purpose a delay is built in into the processing of a sensor scan. This delay is adjustable.
  For scale H0 it should be not larger than 50 and for scale N not larger than 100.
  Default will be 20.

  For each individual decoder a config file needs to be stored in the same directory as the main program.
  For creation of another decoder only a new config file has to be created and than the program
  can be compiled and uploaded
