# Component-tester
Component tester for Arduino:

For documentation, see the Wiki.

Test many types of semiconductors, resistors, capacitors, inductors, etc.

Full list:

Resistors

Capacitors

Inductors

Diodes

Double diodes, common a/c, series, and inverse-parallel

Zever diodes

BJTs (Bipolar Junction Transistors):

    Regular
  
    Darlington pair
  
FETs:

    JFET
  
    MOSFET:
  
        enhancement mode
    
        depletion mode
    
IGBTs (Insulated Gate Bipolar Transistor)

Triacs

Thyristors

Based on Ardutester and earlier works. Uses the ButtonCtl Arduino library (in my Github). Includes hardware design schematic and PCB layout in Eagle format.
The DUT inputs are relay protected against charged capacitors being inserted.
Do not connect batteries or super capacitors.

