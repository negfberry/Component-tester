# Component-tester
Component tester for Arduino:

Test many types of semiconductors, resistors, capacitors, inductors, etc.

Full list:

RESISTORS

CAPACITORS

INDUCTORS

DIODES

BJTS (Bipolar Junction Transistors):

    Regular
  
    Darlington pair
  
FETS:

    JFET
  
    MOSFET:
  
        enhancement mode
    
        depletion mode
    
IGBTS (Insulated Gate Bipolar Transistor)

COMP_TRIACS

THYRISTORS


Based on Ardutester and earlier works. Uses the ButtonCtl Arduino library (in my Github). Includes hardware design schematic and PCB layout in Eagle format.
The DUT inputs are relay protected against charged capacitors being inserted.
Do not connect batteries or super capacitors.

