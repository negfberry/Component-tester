#include <EEPROMVar.h>
#include <EEPROMex.h>
#include <ButtonCtl.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

/*
 *   _______  .__ __         .__          _____  ____________________      ____  _______________  ____ 
 *  \      \ |__|  | ______ |  |        /     \ \_   ___ \__    ___/     /_   |/  _____/\   _  \/_   |
 *  /   |   \|  |  |/ /  _ \|  |       /  \ /  \/    \  \/ |    |  ______ |   /   __  \ /  /_\  \|   |
 * /    |    \  |    <  <_> )  |__    /    Y    \     \____|    | /_____/ |   \  |__\  \\  \_/   \   |
 * \____|__  /__|__|_ \____/|____/    \____|__  /\______  /|____|         |___|\_____  / \_____  /___|
 *        \/        \/                    \/        \/                           \/        \/     
 *        
 * Original Source from:        http:// www.mikrocontroller.net/articles/AVR-Transistortester
 * Original Software:           by Karl-Heinz Kuebbeler (kh_kuebbeler@web.de)
 *                              The Ardutester software is based on porting by Markus Reschke
 *                              (madires@theca-tabellaria.de) 
 *
 * Ardutester version:          PighiXXX (info@pighixxx.com)
 *
 * This version:                Nik Berry (nikberry@gmail.com)
 * Source/Hardware:             https://github.com/negfberry/Component-tester/
 */

 #define VERSION "0.1"
 #define EC 8
 
// #define DEBUG_PRINT                       // Print on Serial Port

// Button press times in deciseconds (from ButtonCtl library)
#define SEC1_PRESS 10                     // Button Long Press
#define SEC5_PRESS 50                     // Button Unsigned Long Long Press (that's a joke)

// UINT32_MAX
#define UINT32_MAX  ((uint32_t)-1)

// Test probes - Must be an ADC port :-)
#define ADC_PORT PORTC                    // ADC port data register
#define ADC_DDR DDRC                      // ADC port data direction register
#define ADC_PIN PINC                      // Port input pins register
#define TP1 0                             // Test pin 1 (=0)
#define TP2 1                             // Test pin 2 (=1)
#define TP3 2                             // Test pin 3 (=2)
#define R_PORT PORTB                      // Port data register
#define R_DDR DDRB                        // Port data direction register
#define TEST_BUTTON 6                     // Test/start push button (active low)
#define DISCHARGE_RELAY 7                 // Discharge relay (active low)
#define TXPIN 2                           // LCD serial transmit

// ADC voltage reference based on Vcc (in mV).
#define VREF_VCC 5001

//  Offset for the internal bandgap voltage reference (in mV): -100 up to 100
//   - To compensate any difference between real value and measured value.
//   - The ADC has a resolution of about 4.88mV for V_ref = 5V (Vcc) and
//     1.07mV for V_ref = 1.1V (bandgap).
//   - Will be added to measured voltage of bandgap reference.
#define VREF_OFFSET 0

//   Exact values of probe resistors.
//    - Standard value for rLow is 680 Ohms.
//    - Standard value for rHigh is 470k Ohms.

// rLow in Ohms
#define R_LOW 680

// rHigh in Ohms
#define R_HIGH 470000

// Offset for systematic error of resistor measurement with rHigh (470k) in Ohms.
#define RH_OFFSET 700

//   Resistance of probe leads (in 0.01 Ohms).
//    - Resistance of two probe leads in series.
//    - Assuming all probe leads got same/similar resistance.
#define R_ZERO 20

//   Capacitance of the wires between PCB and terminals (in pF).
//   Examples:
//    - 2pF for wires 10cm long
#define CAP_WIRES 15

//   Capacitance of the probe leads connected to the tester (in pF).
//   Examples:
//     capacity  length of probe leads
//     -------------------------------
//      3pF      about 10cm
//      9pF      about 30cm
//     15pF      about 50cm

#define CAP_PROBELEADS 9

// Maximum voltage at which we consider a capacitor being discharged (in mV)
#define CAP_DISCHARGED 2
/*
   Number of ADC samples to perform for each mesurement.
    - Valid values are in the range of 1 - 255.
*/
#define ADC_SAMPLES 50

// Estimated internal resistance of port to GND (in 0.1 Ohms)
#define R_MCU_LOW 200                     // Default: 209

// Estimated internal resistance of port to VCC (in 0.1 Ohms)
#define R_MCU_HIGH 220                    // Default: 235

// Voltage offset of µCs analog comparator (in mV): -50 up to 50
#define COMPARATOR_OFFSET 15

// Capacitance of the probe tracks of the PCB and the µC (in pF)
#define CAP_PCB 42

// Total default capacitance (in pF): max. 255
#define C_ZERO CAP_PCB + CAP_WIRES + CAP_PROBELEADS

// ATMEGA328, 16Mhz Related
#define ADC_CLOCK_DIV (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
#define CPU_FREQ F_CPU
#define OSC_STARTUP 16384

// Component ID's
#define COMP_NONE 0
#define COMP_ERROR 1
#define COMP_RESISTOR 10
#define COMP_CAPACITOR 11
#define COMP_INDUCTOR 12
#define COMP_DIODE 20
#define COMP_BJT 21
#define COMP_FET 22
#define COMP_IGBT 23
#define COMP_TRIAC 24
#define COMP_THYRISTOR 25

// Pre-defined LCD special characters
#define LCD_CHAR_OMEGA 244                // Default: 244
#define LCD_CHAR_MICRO 228


// Error type IDs
#define TYPE_DISCHARGE 1                  // Discharge error

// FET type bit masks (also used for IGBTs)
#define TYPE_N_CHANNEL   B00000001        // N-channel
#define TYPE_P_CHANNEL   B00000010        // P-channel
#define TYPE_ENHANCEMENT B00000100        // Enhancement mode
#define TYPE_DEPLETION   B00001000        // Depletion mode
#define TYPE_MOSFET      B00010000        // MOSFET
#define TYPE_JFET        B00100000        // JFET
#define TYPE_IGBT        B01000000        // IGBT (no FET)

// Mode bitmask
#define MODE_LOW_CURRENT   B00000001      // Low test current
#define MODE_HIGH_CURRENT  B00000010      // High test current
#define MODE_DELAYED_START B00000100      // Delayed start

// BJT (bipolar junction transistor) type IDs
#define TYPE_NPN 1                        // NPN
#define TYPE_PNP 2                        // PNP

// Multiplicator tables
#define TABLE_SMALL_CAP 1
#define TABLE_LARGE_CAP 2
#define TABLE_INDUCTOR 3

// Bit flags for pullProbe()
#define FLAG_PULLDOWN B00000000
#define FLAG_PULLUP   B00000001
#define FLAG_1MS      B00001000
#define FLAG_10MS     B00010000

// EEPROM addresses
#define PARAM_RIL 1
#define PARAM_RIH 3
#define PARAM_RZERO 5
#define PARAM_CAPZERO 7
#define PARAM_REFOFFSET 8
#define PARAM_COMPOFFSET 9
#define PARAM_MAGIC 10
#define ROM_MAGIC B10011010

// Tester modes, offsets and values
typedef struct {
  byte samples;                           // Number of ADC samples
  byte refFlag;                           // Internal control flag for ADC
  unsigned int vBandgap;                  // Voltage of internal bandgap reference (mV)
  unsigned int rIntLow;                   // Internal pin resistance of µC in low mode (0.1 Ohms)
  unsigned int rIntHigh;                  // Internal pin resistance of µC in high mode (0.1 Ohms)
  unsigned int rZero;                     // Resistance of probe leads (2 in series) (0.01 Ohms)
  byte capZero;                           // Capacity zero offset (input + leads) (pF)
  signed char refOffset;                  // Voltage offset of bandgap reference (mV)
  signed char compOffset;                 // Voltage offset of analog comparator (mV)
} Parameters;

// Probes
typedef struct {
  // Probe pins
  byte pin1;                              // Probe-1
  byte pin2;                              // Probe-2
  byte pin3;                              // Probe-3
  // Bit masks for switching probes and test resistors
  byte rLow1Mask;                      // rLow mask for probe-1
  byte rHigh1Mask;                     // rHigh mask for probe-1
  byte rLow2Mask;                      // rLow mask for probe-2
  byte rHigh2Mask;                     // rHigh mask for probe-2
  byte rLow3Mask;                      // rLow mask for probe-3
  byte rHigh3Mask;                     // rHigh mask for probe-3
  byte probe1Adc;                       // ADC mask for probe-1
  byte probe2Adc;                       // ADC mask for probe-2
} Probes;

// Checking/probing
typedef struct {
  byte done;                              // Flag for transistor detection done
  byte found;                             // Component type which was found
  byte type;                              // Component specific subtype
  byte resistors;                         // Number of resistors found
  byte diodes;                            // Number of diodes found
  byte probe;                             // Error: probe pin
  unsigned int v;                         // Error: voltage left in mV
} Checks;

// Resistor
typedef struct {
  byte a;                                 // Probe pin #1
  byte b;                                 // Probe pin #2
  byte scale;                             // Exponent of factor (value * 10^x)
  unsigned long value;                    // Resistance
} Resistors;

// Capacitor
typedef struct {
  byte a;                                 // Probe pin #1
  byte b;                                 // Probe pin #2
  signed char scale;                      // Exponent of factor (value * 10^x)
  unsigned long value;                    // Capacitance incl. zero offset
  unsigned long raw;                      // Capacitance excl. zero offset
} Capacitors;

// Inductor
typedef struct {
  signed char scale;                      // Exponent of factor (value * 10^x)
  unsigned long value;                    // Inductance
} Inductors;

// Diode
typedef struct {
  byte a;                                 // Probe pin connected to anode
  byte c;                                 // Probe pin connected to cathode
  unsigned int vF;                       // Forward voltage in mV (high current)
  unsigned int vF2;                      // Forward voltage in mV (low current)
} Diodes;

// Bipolar junction transistor
typedef struct {
  byte b;                                 // Probe pin connected to base
  byte c;                                 // Probe pin connected to collector
  byte e;                                 // Probe pin connected to emitter
  unsigned long hFe;                      // Current amplification factor
  unsigned int iCeo;                     // Leakage current (in µA)
} Bjts;

// FET
typedef struct {
  byte g;                                 // Test pin connected to gate
  byte d;                                 // Test pin connected to drain
  byte s;                                 // Test pin connected to source
  unsigned int vTh;                      // Threshold voltage of gate in mV
} Fets;

// Output buffer
char outBuffer[12];

// Configuration
Parameters parameters;                    // Tester modes, offsets and values

// Probing
Probes probes;                            // Test probes
Checks check;                             // Checking/testing

// Components
Resistors resistors[3];                   // Resistors (3 combinations)
Capacitors caps[3];                       // Capacitors (3 combinations)
Diodes diodes[6];                     // Diodes (3 combinations in 2 directions)
Bjts bjt;                             // Bipolar junction transistor
Fets fet;                             // FET
Inductors inductor;                       // Inductor

byte symLOW1[8]   = { B00000000, B00000000, B00000000, B00010000,
                       B00010010, B00010101, B00010101, B00011010 };

byte symLOW2[8]   = { B00000000, B00000000, B00000000, B00000000,
                       B00010001, B00010001, B00010101, B00001010 };

byte symR[8]      = { B00000000, B00000000, B00011000, B00010100,
                       B00010100, B00011000, B00010100, B00010100 };

byte symF[8]      = { B00000000, B00000000, B00011100, B00010000,
                       B00010000, B00011000, B00010000, B00010000 };

byte symRES1[8]   = { B00000100, B00000100, B00000100, B00000100,
                       B00011111, B00010001, B00010001, B00010001 };

byte symRES2[8]   = { B00010001, B00010001, B00010001, B00010001,
                       B00011111, B00000100, B00000100, B00000100 };

byte symCAP1[8]   = { B00000000, B00000100, B00000100, B00000100,
                       B00000100, B00000100, B00011111, B00011111 };

byte symCAP2[8]   = { B00011111, B00011111, B00000100, B00000100,
                       B00000100, B00000100, B00000100, B00000000 };

byte symFE[8]     = { B00000000, B00000000, B00000000, B00011011,
                       B00010010, B00011011, B00010010, B00010011 };

byte symBE1[8]    = { B00000000, B00000000, B00000000, B00000110,
                       B00000101, B00000110, B00000101, B00000110 };

byte symBE2[8]    = { B00000000, B00000000, B00000000, B00011100,
                       B00010000, B00011100, B00010000, B00011100 };

byte symCE1[8]    = { B00000000, B00000000, B00000000, B00011101,
                       B00010001, B00010001, B00010001, B00011101 };

byte symCE2[8]    = { B00000000, B00000000, B00000000, B00010111,
                       B00000101, B00010101, B00000101, B00010111 };

byte symNPN[8]    = { B00000001, B00010010, B00010100, B00011000,
                       B00011000, B00010100, B00010011, B00000011 };

byte symPNP[8]    = { B00000001, B00010010, B00011100, B00011100,
                       B00011000, B00010100, B00010010, B00000001 };

byte symIND1[8]   = { B00000100, B00000100, B00000100, B00000100,
                       B00011111, B00011111, B00011111, B00011111 };

byte symIND2[8]   = { B00011111, B00011111, B00011111, B00011111,
                       B00011111, B00000100, B00000100, B00000100 };

byte symDIODE1[8] = { B00000100, B00000100, B00000100, B00011111,
                       B00001110, B00001110, B00000100, B00011111 };

#define symTHY1 symDIODE1

byte symDIODE2[8] = { B00000100, B00000100, B00000100, B00000100,
                       B00000000, B00000000, B00000000, B00000000 };

byte symTHY2[8]   = { B00000100, B00001100, B00010100, B00000100,
                       B00000000, B00000000, B00000000, B00000000 };

byte symTHY3[8]   = { B00000000, B00000000, B00011111, B00000000,
                       B00000000, B00000000, B00000000, B00000000 };

byte symTRI1[8]   = { B00000001, B00000001, B00000001, B00011111,
                       B00000100, B00001110, B00001110, B00011111 };

byte symTRI2[8]   = { B00000000, B00000000, B00000000, B00011111,
                       B00001110, B00001110, B00000100, B00011111 };

byte symTRI3[8]   = { B00000101, B00000101, B00001001, B00011001,
                       B00000001, B00000001, B00000001, B00000001 };

byte symFet[8]    = { B00000010, B00010010, B00011110, B00010000,
                       B00010000, B00011110, B00010010, B00000010 };

byte symNFetG[8]  = { B00000000, B00000000, B00000000, B00000000,
                       B00000010, B00011111, B00000010, B00000000 };

byte symPFetG[8]  = { B00000000, B00000010, B00011111, B00000010,
                       B00000000, B00000000, B00000000, B00000000 };

byte symPFetD[8]  = { B00000010, B00000010, B00000010, B00011000,
                       B00010100, B00010100, B00010100, B00011000 };

byte symNFetD[8]  = { B00011000, B00010100, B00010100, B00010100,
                       B00011000, B00000010, B00000010, B00000010 };

byte symPFetS[8]  = { B00001100, B00010000, B00001000, B00000100,
                       B00011000, B00000010, B00000010, B00000010 };

byte symNFetS[8]  = { B00000010, B00000010, B00000010, B00001100,
                       B00010000, B00001000, B00000100, B00011000 };

byte symJFetT[8]  = { B00000000, B00000000, B00000000, B00000000,
                       B00000000, B00000010, B00000010, B00000010 };

byte symJFetB[8]  = { B00000010, B00000010, B00000010, B00000000,
                       B00000000, B00000000, B00000000, B00000000 };

byte symH[8]      = { B00010000, B00010110, B00011001, B00010001,
                       B00010001, B00010001, B00000000, B00000000 };

byte symV[8]      = { B00010001, B00010001, B00010001, B00010001,
                       B00001010, B00000100, B00000000, B00000000 };

byte symI[8]      = { B00001110, B00000100, B00000100, B00000100,
                       B00000100, B00001110, B00000000, B00000000 };

byte symIGBT1[8]  = { B00000000, B00000000, B00000000, B00000000,
                       B00000000, B00000000, B00000001, B00000010 };

byte symIGBT2[8]  = { B00010000, B00010000, B00010000, B00010000,
                       B00010000, B00010000, B00011100, B00000100 };

byte symIGBT3[8]  = { B00000000, B00000001, B00000001, B00000001,
                       B00000001, B00000001, B00011111, B00000000 };

byte symIGBT4[8]  = { B00010010, B00010100, B00011000, B00010000,
                       B00010000, B00011000, B00010100, B00000010 };

byte symIGBT5[8]  = { B00000100, B00000100, B00011111, B00000100,
                       B00001110, B00001110, B00011111, B00000100 };

byte symIGBT6[8]  = { B00000011, B00000011, B00000000, B00000000,
                       B00000000, B00000000, B00000000, B00000000 };

byte symIGBT7[8]  = { B00000100, B00000100, B00011100, B00010000,
                       B00010000, B00010000, B00010000, B00010000 };

// Prefix Table
//                                     -12  -9   -6              -3   0  +3   +6
const unsigned char prefixTable[]  = {'p', 'n', LCD_CHAR_MICRO, 'm', 0, 'k', 'M'};

// Voltage based factors for large caps (using R low)
const unsigned int largeCapTable[]  = { 23022, 21195, 19629, 18272, 17084,
                                         16036, 15104, 14271, 13520, 12841,
                                         12224, 11660, 11143, 10668, 10229,
                                         9822, 9445, 9093, 8765, 8458, 8170,
                                         7900, 7645, 7405, 7178, 6963, 6760,
                                         6567, 6384, 6209, 6043, 5885, 5733,
                                         5589, 5450, 5318, 5191, 5069, 4952,
                                         4839, 4731, 4627, 4526, 4430, 4336 };

// Voltage based factors for small caps (using R high)
const unsigned int smallCapTable[]  = { 954, 903, 856, 814, 775, 740, 707, 676, 648 };

// Ratio based factors for inductors
const unsigned int inductorTable[]  = { 4481, 3923, 3476, 3110, 2804, 2544,
                                         2321, 2128, 1958, 1807, 1673, 1552,
                                         1443, 1343, 1252, 1169, 1091, 1020,
                                         953, 890, 831, 775, 721, 670, 621,
                                         574, 527, 481, 434, 386, 334, 271 };

// Bitmasks for R low probe resistors based on probe ID
const unsigned char rLowTable[]  = { (1 << (TP1 * 2)), (1 << (TP2 * 2)), (1 << (TP3 * 2)) };

// Bitmasks for ADC pins based on probe ID
const unsigned char adcTable[]  = { (1 << TP1), (1 << TP2), (1 << TP3) };

SoftwareSerial lcd(-1, TXPIN);
ButtonCtl testButton(TEST_BUTTON);

// Define character, Newhaven Displays LCD
// Will convert later to 4WireLCD

void lcd_chardef(byte c) {
  lcd.write(0xFE);
  lcd.write(0x54);
  lcd.write((byte) c);
}

void lcd_createChar(byte i, byte *icon) {
  byte j;

  lcd_chardef(i);
  for(j = 0; j < 8; j++) lcd.write((byte) icon[j]);
  delay(100);
}

void lcd_clear() {
  lcd.write(0xFE);
  lcd.write(0x51);
  delay(2);
}

void lcd_home() {
  lcd_setcursor(0, 0);
}

// Moves the LCD cursor to x,y
void lcd_setcursor(int x, int y) {
  byte pos;

  switch (y) {
    case 0:
      pos = 0x00;
      break;
    case 1:
      pos = 0x40;
      break;
    case 2:
      pos = 0x14;
      break;
    case 3:
      pos = 0x54;
      break;
  }
  pos += x;
  lcd.write(0xFE);
  lcd.write(0x45);
  lcd.write(pos);
}

void lcd_backlight(byte b) {
  static byte o;
  byte r = 4;

  if(o == b) return;
  o = b;
  switch (b) {
    case 0:
      r = 4;
      break;
    case 1:
      r = 6;
      break;
    case 2:
      r = 8;
      break;
  }
  lcd.write(0xFE);
  lcd.write(0x53);
  lcd.write(r);
  delay(1);
}

void lcd_contrast(int c) {
  static byte o;
  byte r = 40;

  if(o == c) return;
  o = c;
  switch (c) {
    case 0:
      r = 30;
      break;
    case 1:
      r = 35;
      break;
    case 2:
      r = 40;
      break;
    case 3:
      r = 50;
      break;
  }
  lcd.write(0xFE);
  lcd.write(0x52);
  lcd.write(r);
  delay(1);
}

void lcd_setbaud(long int b) {
  byte r;
  switch (b) {
    case 2400:
      r = 11;
      break;
    case 4800:
      r = 12;
      break;
    case 9600:
      r = 13;
      break;
    case 14400:
      r = 14;
      break;
    case 19200:
      r = 15;
      break;
    case 38400:
      r = 16;
      break;
  }
  lcd.write(0x7C);
  lcd.write(r);
}


// Setup function
void setup() {
  byte test;                              // Test value

  pinMode(DISCHARGE_RELAY, OUTPUT);
  digitalWrite(DISCHARGE_RELAY, HIGH);
  lcd.begin(9600);
  lcd_clear();

#ifdef DEBUG_PRINT
  Serial.begin(9600);                     // Serial Output
#endif

  // Setup µC
  ADCSRA = (1 << ADEN) | ADC_CLOCK_DIV;   // Enable ADC and set clock divider
  MCUSR &= ~(1 << WDRF);                  // Reset watchdog flag
  DIDR0 = B00110111;                      // DIDR0 Digital Input Disable Register 0
                                          // - Disable digital input on analog pins
  wdt_disable();                          // Disable watchdog
  // Default offsets and values
  parameters.samples = ADC_SAMPLES;           // Number of ADC samples
  parameters.refFlag = 1;                     // No ADC reference set yet
  delay(100);

  // Reset variables
  analogReference(EXTERNAL);              // Set Analog Reference to External

  // Init
  loadAdjust();                           // Load adjustment values

#ifdef DEBUG_PRINT
  Serial.print(F("Nikol MCT-1601"));
  Serial.println();
  Serial.println(F("Based on Ardutester by PighiXXX & PaoloP"));
  Serial.println(F("from original version by Markus Reschke"));
  Serial.println();
  Serial.println(F("Press Button to Probe"));
  Serial.println(F("Long press to adjust and save"));
#endif
}

byte gogohut = 0;

// Main loop
void loop() {
  byte test;

  pinMode(TEST_BUTTON, INPUT_PULLUP);
  // Reset variables
  check.found = COMP_NONE;
  check.type = 0;
  check.done = 0;
  check.diodes = 0;
  check.resistors = 0;
  bjt.hFe = 0;
  bjt.iCeo = 0;
  // Reset hardware
  setAdcHighZ();                            // Set all pins of ADC port as input
  if(!gogohut) {
    lcd_clear();
    lcd.print(F("   Nikol MCT-1601"));
    lcd_setcursor(0, 1);
    lcd.print(F("Press button to test"));
    lcd_setcursor(0, 2);
    lcd.print(F("Hold 1s cont. test"));
    lcd_setcursor(0, 3);
    lcd.print(F("Hold 5s to calibrate"));
  }
  
  // Internal bandgap reference
  parameters.vBandgap = readVoltage(0x0e, 1);      // Dummy read for bandgap stabilization
  parameters.vBandgap = readVoltage(0x0e, 1);      // Get voltage of bandgap reference
  parameters.vBandgap += parameters.refOffset;   // Add voltage offset
  digitalWrite(DISCHARGE_RELAY, HIGH);    // Turn off relay (discharge DUT)
  test = testKey();
  if(test == 2) {                         // Long Press
    wdt_disable();                        // Disable watchdog
    adjustAndSave();                      // Calibrate
  } else {
    if(!gogohut) lcd_clear();
    if(allProbesShorted() == 3) {         // All probes Shorted!
#ifdef DEBUG_PRINT
        Serial.println();
#endif
        lcd.print(F("Remove"));
        lcd_setcursor(0, 1);
        lcd.print(F("Short Circuit"));
      } else {
        if(!gogohut) lcd_setcursor(0, 1);              // Move to line #2
        // Display start of probing
        digitalWrite(DISCHARGE_RELAY, LOW); // Turn on relay (connect DUT to tester)
        if(!gogohut) lcd.print(F("Discharging DUT..."));
        dischargeProbes();
        if(!gogohut) lcd_setcursor(0, 2);              // Move to line #2
        if(!gogohut) lcd.print(F("Testing..."));    // Display: Testing...
        if(check.found == COMP_ERROR) {   // Discharge failed
          lcd.print(F("Battery?"));
          // Display probe number and remaining voltage
          lcd_setcursor(0, 2);
          lcd_testpin(check.probe);
          lcd.write(':');
          lcd.write(' ');
          displayValue(check.v, -3, 'V');
        } else {                          // Skip all other checks
          // Check all 6 combinations of the 3 probes
          checkProbes(TP1, TP2, TP3);
          checkProbes(TP2, TP1, TP3);
          checkProbes(TP1, TP3, TP2);
          checkProbes(TP3, TP1, TP2);
          checkProbes(TP2, TP3, TP1);
          checkProbes(TP3, TP2, TP1);
          // If component might be a capacitor
          if(check.found == COMP_NONE || check.found == COMP_RESISTOR) {
#ifdef DEBUG_PRINT
            Serial.println();
            Serial.println(F("Wait a moment..."));
#endif
            // Tell user to be patient with large caps
            if(!gogohut)  {
              lcd_clear_line(2);
              lcd.print(F("Testing..."));
              lcd.write(' ');
              lcd.write('C');
            }
            // Check all possible combinations
            measureCap(TP3, TP1, 0);
            measureCap(TP3, TP2, 1);
            measureCap(TP2, TP1, 2);
          }
          lcd_clear();
          // Call output function based on component type
#ifdef DEBUG_PRINT
          Serial.print(F("Found: "));
          // Components ID's
          switch (check.found) {
            case COMP_ERROR:
              Serial.println(F("Component Error!"));
              break;

            case COMP_NONE:
              Serial.println(F("No Component!"));
              break;

            case COMP_RESISTOR:
              Serial.println(F("Resistor"));
              break;

            case COMP_CAPACITOR:
              Serial.println(F("Capacitor"));
              break;

            case COMP_INDUCTOR:
              Serial.println(F("Inductor"));
              break;

            case COMP_DIODE:
              Serial.println(F("Diode"));
              break;

            case COMP_BJT:
              Serial.println(F("BJT"));
              break;

            case COMP_FET:
              Serial.println(F("FET"));
              break;

            case COMP_IGBT:
              Serial.println(F("IGBT"));
              break;

            case COMP_TRIAC:
              Serial.println(F("TRIAC"));
              break;

            case COMP_THYRISTOR:
              Serial.println(F("Thyristor"));
              break;
          }
#endif
          lcd_clear();
          switch (check.found) {
            case COMP_ERROR:
              showError();
              break;

            case COMP_DIODE:
              showDiode();
              break;

            case COMP_BJT:
              showBjt();
              break;

            case COMP_FET:
              showFet();
              break;

            case COMP_IGBT:
              showIGBT();
              break;

            case COMP_THYRISTOR:
              showSpecial();
              break;

            case COMP_TRIAC:
              showSpecial();
              break;

            case COMP_RESISTOR:
              showResistor();
              break;

            case COMP_CAPACITOR:
              showCapacitor();
              break;

            default:                      // No component found
              showFail();
          }
        }
     }
  }
  testKey();                            // Let the user read the text
  wdt_disable();                          // Disable watchdog
}

// Set ADC port to HiZ mode
void setAdcHighZ(void) {
  ADC_DDR &= ~(1<<TP1);
  ADC_DDR &= ~(1<<TP2);
  ADC_DDR &= ~(1<<TP3);
}

// Set ADC port low
void setAdcLow(void) {
  ADC_PORT &= ~(1<<TP1);
  ADC_PORT &= ~(1<<TP2);
  ADC_PORT &= ~(1<<TP3);
}

// Setup probes, bitmasks for probes and test resistors
void updateProbes(byte probe1, byte probe2, byte probe3) {
  // DSt probe IDs
  probes.pin1 = probe1;
  probes.pin2 = probe2;
  probes.pin3 = probe3;
  // Setup masks using bitmask tables
  probes.rLow1Mask = rLowTable[probe1];
  probes.rHigh1Mask = probes.rLow1Mask + probes.rLow1Mask;
  probes.probe1Adc = adcTable[probe1];
  probes.rLow2Mask = rLowTable[probe2];
  probes.rHigh2Mask = probes.rLow2Mask + probes.rLow2Mask;
  probes.probe2Adc = adcTable[probe2];
  probes.rLow3Mask = rLowTable[probe3];
  probes.rHigh3Mask = probes.rLow3Mask + probes.rLow3Mask;
}

// Check for a short circuit between two probes
byte shortedProbes(byte probe1, byte probe2) {
  byte flag = 0;                          // Return value
  unsigned int v1;                        // Voltage at probe #1 in mV
  unsigned int v2;                        // Voltage at probe #2 in mV

//   Set up a voltage divider between the two probes:
//    - probe1: rLow pull-up
//    - probe2: rLow pull-down
//    - third probe: HiZ

  R_PORT = rLowTable[probe1];
  R_DDR = rLowTable[probe1] | rLowTable[probe2];
  // Read voltages
  v1 = readVoltage(probe1, 1);
  v2 = readVoltage(probe2, 1);

//   We expect both probe voltages to be about the same and
//   to be half of Vcc (allowed difference +/- 30mV).

  if((v1 > VREF_VCC / 2 - 30) && (v1 < VREF_VCC / 2 + 30)
   &&(v2 > VREF_VCC / 2 - 30) && (v2 < VREF_VCC / 2 + 30)) flag = 1;

  // Reset port
  R_DDR = 0;
  return flag;
}

 // Check for a short circuit between all probes
byte allProbesShorted(void) {
  byte flag;                              // Return value

  // Check all possible combinations
  flag = shortedProbes(TP1, TP2);
  flag += shortedProbes(TP1, TP3);
  flag += shortedProbes(TP2, TP3);
  return flag;
}

// Try to discharge any connected components, e.g. capacitors
// The discharge relay should already have accomplished this,
// the first time it's called, so in that case this is more of
// a test that it worked. Subsequent calls really do something.
void dischargeProbes(void) {
  byte cnt;                           // Loop control
  byte limit = 40;                        // Sliding timeout (2s)
  byte id;                                // Test pin
  byte dischargeMask;                     // Bitmask
  unsigned int vNow;                       // Current voltage
  unsigned int vOld[3];                  // Old voltages

  // Set probes to a save discharge mode (pull-down via rHigh)
  // Set ADC port to HiZ input
  setAdcHighZ();
  setAdcLow();
  // All probe pins: rHigh and rLow pull-down
  R_PORT = 0;
  R_DDR = (2 << (TP1 * 2)) | (2 << (TP2 * 2)) | (2 << (TP3 * 2));
  R_DDR |= (1 << (TP1 * 2)) | (1 << (TP2 * 2)) | (1 << (TP3 * 2));
  // Get voltages
  vOld[0] = readVoltage(TP1, 1);
  vOld[1] = readVoltage(TP2, 1);
  vOld[2] = readVoltage(TP3, 1);

//   Try to discharge probes
//    - We check if the voltage decreases over time.
//    - A slow discharge rate will increase the timeout to support
//      large caps.
//    - A very large cap will discharge too slowly and an external voltage
//      maybe never :-)

  cnt = 1;
  id = 2;
  dischargeMask = 0;
  while (cnt > 0) {
    id++;                                 // Next probe
    if(id > 2) id = 0;                    // Start with probe #1 again
    if(dischargeMask & (1 << id))         // Skip discharged probe
      continue;
    vNow = readVoltage(id, 1);                   // Get voltage of probe
    if(vNow < vOld[id]) {                 // Voltage decreased
      vOld[id] = vNow;                    // Update old value
      // Adapt timeout based on discharge rate
      if((limit - cnt) < 20) {
        // Increase timeout while preventing overflow
        if(limit < (255 - 20)) limit += 20;
      }
      cnt = 1;                        // Reset no-changes counter
    } else {                              // Voltage not decreased
      // Increase limit if we start at a low voltage
      if((vNow < 10) && (limit <= 40)) limit = 80;
      cnt++;                          // Increase no-changes counter
    }
    if(vNow <= CAP_DISCHARGED) {           // Seems to be discharged
      dischargeMask |= (1 << id);         // Set flag
    } else if(vNow < 800) {                // Extra pull-down
      // It's safe now to pull-down probe pin directly
      ADC_DDR |= adcTable[id];
    }
    if(dischargeMask == B00000111) {      // All probes discharged
      cnt = 0;                        // End loop
    } else if(cnt > limit) {          // No decrease for some time
      // Might be a battery or a super cap
      check.found = COMP_ERROR;           // Report error
      check.type = TYPE_DISCHARGE;        // Discharge problem
      check.probe = id;                   // Save probe
      check.v = vNow;                      // Save voltage
      cnt = 0;                        // End loop
    } else {                              // Go for another round
      wdt_reset();                        // Reset watchdog
      delay(50);                          // Wait for 50ms
    }
  }
  // Reset probes
  R_DDR = 0;                              // Set resistor port to input mode
  setAdcHighZ();                            // Set ADC port to input mode
}

// Pull probe up/down via probe resistor for 1 or 10 ms
void pullProbe(byte Mask, byte mode) {
  // Set pull mode
  if(mode & FLAG_PULLUP) R_PORT |= Mask;  // Pull-up
  else R_PORT &= ~Mask;                   // Pull-down
  R_DDR |= Mask;                          // Enable pulling
  if(mode & FLAG_1MS) delay(1);           // Wait 1ms
  else delay(10);                         // Wait 10ms
  // Reset pulling
  R_DDR &= ~Mask;                         // Set to HiZ mode
  R_PORT &= ~Mask;                        // Set 0
}

// Rescale value
unsigned long RescaleValue(unsigned long value, signed char scale, signed char newScale)
{
  unsigned long newValue;
  newValue = value;                       // Take old value
  while (scale != newScale) {             // Processing loop
    if(newScale > scale) {                // Upscale
      newValue /= 10;
      scale++;
    } else {                              // Downscale
      newValue *= 10;
      scale--;
    }
  }
  return newValue;
}

// Lookup a voltage/ratio based factor in a table and interpolate its value
unsigned int getFactor(unsigned int vIn, byte id) {
  unsigned int factor;                    // Return value
  unsigned int vDiff;                    // Voltage difference to table start
  unsigned int fact1, fact2;              // Table entries
  unsigned int tabStart;                  // Table start voltage
  unsigned int tabStep;                   // Table step voltage
  unsigned int tabIndex;                  // Table entries (-2)
  unsigned int *table;
  byte index;                             // Table index
  byte diff;                              // Difference to next entry

  // Setup table specific stuff
  if(id == TABLE_SMALL_CAP) {
    tabStart = 1000;                      // Table starts at 1000mV
    tabStep = 50;                         // 50mV steps between entries
    tabIndex = 7;                         // Entries in table - 2
    table = (unsigned int *) &smallCapTable[0];  // Pointer to table start
  } else if(id == TABLE_LARGE_CAP) {
    tabStart = 300;                       // Table starts at 1000mV
    tabStep = 25;                         // 25mV steps between entries
    tabIndex = 42;                        // Entries in table - 2
    table = (unsigned int *) &largeCapTable[0];  // Pointer to table start
  } else if(id == TABLE_INDUCTOR) {
    tabStart = 200;                       // Table starts at 200
    tabStep = 25;                         // Steps between entries
    tabIndex = 30;                        // Entries in table - 2
    table = (unsigned int *) &inductorTable[0];  // Pointer to table start
  } else return 0;
  // We interpolate the table values corresponding to the given voltage/ratio,
  // difference to start of table
  if(vIn >= tabStart) vDiff = vIn - tabStart;
  else vDiff = 0;
  // Calculate table index
  index = vDiff / tabStep;               // Index (position in table)
  diff = vDiff % tabStep;                // Difference to index
  diff = tabStep - diff;                  // Difference to next entry
  // Prevent index overflow
  if(index > tabIndex) index = tabIndex;
  // Get values for index and next entry
  table += index;                         // Advance to index
  fact1 = *(table);
  table++;                                // Next entry
  fact2 = *(table);
  // Interpolate values based on the difference
  factor = fact1 - fact2;
  factor *= diff;
  factor += tabStep / 2;
  factor /= tabStep;
  factor += fact2;
  return factor;
}

// Identify component
void checkProbes(byte probe1, byte probe2, byte probe3) {
  byte flag;                              // Temporary value
  unsigned int vRLow;                      // Voltage across R low (load)
  unsigned int v1;                       // Voltage #1

  // Init
  if(check.found == COMP_ERROR) return;   // Skip check on any error
  wdt_reset();                            // Reset watchdog
  updateProbes(probe1, probe2, probe3);   // Update bitmasks

//   We measure the current from probe 2 to ground with probe 1 pulled up
//   to 5V and probe 3 in HiZ mode to determine if we got a self-conducting
//   part, i.e. diode, resistor or depletion-mode FET. R low is used as current
//   shunt.
// 
//   In case of a FET we have to take care about the gate charge based on
//   the channel type.

  // Set probes: Gnd -- R low -- probe-2 / probe-1 -- Vcc
  R_PORT = 0;                             // Set resistor port to Gnd
  R_DDR = probes.rLow2Mask;            // Pull down probe-2 via R low
  ADC_DDR = probes.probe1Adc;           // Set probe-1 to output
  ADC_PORT = probes.probe1Adc;          // Pull-up probe-1 directly

//   For a possible n channel FET we pull down the gate for a few ms,
//   assuming: probe-1 = D / probe-2 = S / probe-3 = G

  // Discharge gate via R low
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLDOWN);
  vRLow = readVoltage5ms(probes.pin2);         // Get voltage at R low

//   If we got conduction we could have a p channel FET. For any
//   other part vRLow will be the same.

  if(vRLow >= 977) {                       // > 1.4mA

//     For a possible p channel FET we pull up the gate for a few ms,
//     assuming: probe-1 = S / probe-2 = D / probe-3 = G

    // Discharge gate via R low
    pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLUP);
    vRLow = readVoltage5ms(probes.pin2);       // Get voltage at R low
  }

//   If there's some current we could have a depletion-mode FET
//   (self-conducting). To skip germanium BJTs with a high leakage current
//   we check for a current larger then the usual V_CEO.
// 
//   Other possibilities:
//    - diode or resistor

  if(vRLow > 490) {                        // > 700µA (was 92mV/130µA)
    checkDepletionModeFet(vRLow);
  }

//   If there's nearly no conduction (just a small leakage current) between
//   probe-1 and probe-2 we might have a semiconductor:
//    - BJT
//    - enhancement mode FET
//    - Thyristor or Triac
//    or a large resistor

  if(vRLow < 977) {                        // Load current < 1.4mA

//     check for:
//      - PNP BJT (common emitter circuit)
//      - p-channel MOSFET (low side switching circuit)

    if(check.done == 0) {                 // Not sure yet
      // We assume: probe-1 = E / probe-2 = C / probe-3 = B,
      // set probes: Gnd -- R low - probe-2 / probe-1 -- Vcc
      R_DDR = probes.rLow2Mask;        // Enable R low for probe-2
      R_PORT = 0;                         // Pull down collector via R low
      ADC_DDR = probes.probe1Adc;       // Set probe 1 to output
      ADC_PORT = probes.probe1Adc;      // Pull up emitter directly
      delay(5);
      R_DDR = probes.rLow2Mask | probes.rLow3Mask; // Pull down base via R low
      v1 = readVoltage5ms(probe2);            // Get voltage at collector
      // If DUT is conducting we might have a PNP BJT or p-channel FET.
      if(v1 > 3422) {                    // Detected current > 4.8mA
        // Distinguish PNP BJT from p-channel MOSFET
        checkBjtMosFet(TYPE_PNP, vRLow);
      }
    }

//     Check for
//      - NPN BJT (common emitter circuit)
//      - Thyristor and Triac
//      - n-channel MOSFET (high side switching circuit)

    if(check.done == 0) {                 // Not sure yet
      // We assume: probe-1 = C / probe-2 = E / probe-3 = B,
      // set probes: Gnd -- probe-2 / probe-1 -- R low -- Vcc
      ADC_DDR = probes.probe2Adc;       // Set probe-2 to output mode
      setAdcLow();                        // Pull down probe-2 directly
      R_DDR = probes.rLow1Mask | probes.rLow3Mask; // Select R low for probe-1 & R low for probe-3
      R_PORT = probes.rLow1Mask | probes.rLow3Mask; // Pull up collector & base via R low
      v1 = readVoltage5ms(probe1);            // Get voltage at collector
      // If DUT is conducting we might have a NPN BJT, something similar or a n-channel MOSFET.
      if(v1 < 1600) {                    // Detected current > 4.8mA
        // First check for thyristor and triac
        flag = checkThyristorTriac();
        if(flag == 0) {                   // No thyristor or triac
          // We might got a NPN BJT or a n-channel MOSFET.
          checkBjtMosFet(TYPE_NPN, vRLow);
        }
      }
    }

//   If there's conduction between probe-1 and probe-2 we might have a
//    - diode (conducting)
//    - small resistor (checked later on)

  } else {                                // Load current > 1.4mA
    // We check for a diode even if we already found a component to get Vf,
    // since there could be a body/protection diode of a transistor.
    checkDiode();
  }
  // Check for a resistor.
  if(check.found == COMP_NONE || check.found == COMP_RESISTOR) {
    checkResistor();
  // Otherwise run some final checks.
  } else {
    // Verify a MOSFET
    if(check.found == COMP_FET && check.type & TYPE_MOSFET)
      verifyMosFet();
  }
  // Clean up
  setAdcHighZ();                            // Set ADC port to HiZ mode
  setAdcLow();                            // Set ADC port low
  R_DDR = 0;                              // Set resistor port to HiZ mode
  R_PORT = 0;                             // Set resistor port low
}

// Read ADC and return voltage in mV
unsigned int readVoltage(byte Probe, byte scale)
{
  unsigned int v;                         // Return value (mV)
  byte lc;                                // Loop counter
  unsigned long int value;                // ADC value

  for(lc = 0, value = 0; lc < parameters.samples; lc++) {
    value += doReadV(Probe, scale);              // Add ADC reading
  }
  // De-sample to get average voltage
  value /= parameters.samples;
  v = value;
  return v;
}

// Read ADC and return voltage in mV
unsigned int doReadV(byte probe, byte scale)
{
  unsigned int v;                         // Return value (mV)
  byte lc;                                // Loop counter
  unsigned long int value;                // ADC value
  probe |= (1 << REFS0);                  // Use internal reference anyway
  char i;
  byte saveRef;

  for(i = 0; i < 1; i++) {
    ADMUX = probe;                        // Set input channel and v reference

    // If voltage reference has changed, run a dummy conversion
    // (recommended by datasheet)

    saveRef = probe & (1 << REFS1);       // Get REFS1 bit flag
    if(saveRef != parameters.refFlag) {
      delayMicroseconds(100);             // Time for voltage stabilization
      ADCSRA |= (1 << ADSC);              // Start conversion
      while (ADCSRA & (1 << ADSC));       // Wait until conversion is done
      parameters.refFlag = saveRef;           // Update flag
    }
    // Sample ADC readings
    value = 0;                            // Reset sampling variable
    ADCSRA |= (1 << ADSC);              // Start conversion
    while (ADCSRA & (1 << ADSC));       // Wait until conversion is done
    value += ADCW;                      // Add ADC reading
    // Auto-switch voltage reference for low readings
    if(lc == 4 && value < 1024 && !(probe & (1 << REFS1)) && scale == 1) {
      probe |= (1 << REFS1);            // Select internal bandgap reference
      i = -1;                           // Re-run sampling
      break;
    }
  }
  // Convert ADC reading to voltage - single sample: v = ADC reading * vRef / 1024
  // Get voltage of reference used
  if(probe & (1 << REFS1)) v = parameters.vBandgap; // Bandgap reference
  else v = VREF_VCC;                      // Vcc reference
  // Convert to voltage
  value *= v;                             // ADC readings * vRef
  value /= 1024;                          // / 1024 for 10bit ADC
  v = value;
  return v;
}

// Wait 5ms and then read ADC
unsigned int readVoltage5ms(byte Probe) {
   delay(5);                              // Wait 5ms
   return (readVoltage(Probe, 1));
}

// Wait 20ms and then read ADC
unsigned int readVoltage20ms(byte Probe) {
  delay(20);                              // Wait 20ms
  return (readVoltage(Probe, 1));
}

// Measure hFE of BJT in common collector circuit (emitter follower)
unsigned long getHFeCommonCollector(byte type) {
  unsigned long hFe;                      // Return value
  unsigned int vREmitter;                     // Voltage across emitter resistor
  unsigned int vRBase;                     // Voltage across base resistor
  unsigned int rInternal;                        // Internal resistance of µC

//   Measure hFE for a BJT in common collector circuit
//   (emitter follower):
//    - hFE = (I_e - I_b) / I_b
//    - measure the voltages across the resistors and calculate the currents
//      (resistor values are well known)
//    - hFE = ((vREmitter / rEmitter) - (vRBase / rBase)) / (vRBase / rBase)

  // Setup probes and get voltages
  if(type == TYPE_NPN) {                  // NPN
    // We assume: probe-1 = C / probe-2 = E / probe-3 = B,
    // set probes: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
    ADC_DDR = probes.probe1Adc;         // Set probe 1 to output
    ADC_PORT = probes.probe1Adc;        // Pull up collector directly
    R_DDR = probes.rLow2Mask | probes.rLow3Mask; // Select rLow for probe-2 & rLow for probe-3
    R_PORT = probes.rLow3Mask;         // Pull up base via rLow
    vREmitter = readVoltage5ms(probes.pin2);      // vREmitter = ve
    vRBase = VREF_VCC - readVoltage(probes.pin3, 1); // vRBase = Vcc - vb
  } else {                                // PNP
    // We assume: probe-1 = E / probe-2 = C / probe-3 = B,
    // set probes: Gnd -- probe-2 / probe-1 -- rLow -- Vcc
    setAdcLow();                          // Set ADC port low
    ADC_DDR = probes.probe2Adc;         // Pull down collector directly
    R_PORT = probes.rLow1Mask;         // Pull up emitter via rLow
    R_DDR = probes.rLow1Mask | probes.rLow3Mask; // Pull down base via rLow
    vREmitter = VREF_VCC - readVoltage5ms(probes.pin1); // vREmitter = Vcc - ve
    vRBase = readVoltage(probes.pin3, 1);       // vRBase = vb
  }
  if(vRBase < 10) {                        // I_b < 14µA -> Darlington
    // Change base resistor from rLow to rHigh and measure again
    if(type == TYPE_NPN) {                // NPN
      R_DDR = probes.rLow2Mask | probes.rHigh3Mask; // Select rLow for probe-2 & rHigh for probe-3
      R_PORT = probes.rHigh3Mask;      // Pull up base via rHigh
      vREmitter = readVoltage5ms(probes.pin2);    // vREmitter = ve
      vRBase = VREF_VCC - readVoltage(probes.pin3, 1); // vRBase = Vcc - vb
      rInternal = parameters.rIntLow;                    // Get internal resistor
    } else {                              // PNP
      R_DDR = probes.rLow1Mask | probes.rHigh3Mask; // Pull down base via rHigh
      vREmitter = VREF_VCC - readVoltage5ms(probes.pin1); // vREmitter = Vcc - ve
      vRBase = readVoltage(probes.pin3, 1);     // vRBase = vb
      rInternal = parameters.rIntHigh;                    // Get internal resistor
    }

//     Since I_b is so small vs. I_e we'll neglect it and use
//      hFE = I_e / I_b
//          = (vREmitter / rEmitter) / (vRBase / rBase)
//          = (vREmitter * rBase) / (vRBase * rEmitter)

    if(vRBase < 1) vRBase = 1;              // Prevent division by zero
    hFe =  vREmitter * R_HIGH;                // vREmitter * rBase
    hFe /= vRBase;                         // / vRBase
    hFe *= 10;                            // Upscale to 0.1
    hFe /= (R_LOW * 10) + rInternal;             // / rEmitter in 0.1 Ohm
  } else {                                // I_b > 14µA -> standard

//     Both resistors are the same (rEmitter = rBase):
//      - hFE = ((vREmitter / rEmitter) - (vRBase / rBase)) / (vRBase / rBase)
//      -     = (vREmitter - vRBase) / vRBase

    hFe = (vREmitter - vRBase) / vRBase;
  }
  return hFe;
}

// Measure the gate threshold voltage of a depletion-mode MOSFET
void getGateThreshold(byte type) {
  unsigned long vTh = 0;                  // Gate threshold voltage
  byte drainRLow;                          // rLow bitmask for drain
  byte drainAdc;                         // ADC bitmask for drain
  byte pullMode;
  byte lc;                                // Loop counter

   // Init variables
  if(type & TYPE_N_CHANNEL) {             // N-channel

//    We assume: probe-1 = D / probe-2 = S / probe-3 = G
//     probe-2 is still pulled down directly
//     probe-1 is still pulled up via rLow

    drainRLow =  probes.rLow1Mask;
    drainAdc = probes.probe1Adc;
    pullMode = FLAG_10MS | FLAG_PULLDOWN;
  } else {                                // P-channel

//   We assume: probe-1 = S / probe-2 = D / probe-3 = G
//    probe-2 is still pulled down via rLow
//    probe-1 is still pulled up directly

    drainRLow =  probes.rLow2Mask;
    drainAdc = probes.probe2Adc;
    pullMode = FLAG_10MS | FLAG_PULLUP;
  }
  // For low reaction times we use the ADC directly.
  // Sanitize bit mask for drain to prevent a never-ending loop
  drainAdc &= B00000111;                 // drain
  ADMUX = probes.pin3 | (1 << REFS0);    // Select probe-3 for ADC input
  // Sample 10 times
  for (lc = 0; lc < 10; lc++) {
    wdt_reset();                          // Reset watchdog
    // Discharge gate via rLow for 10 ms
    pullProbe(probes.rLow3Mask, pullMode);
    // Pull up/down gate via rHigh to slowly charge gate
    R_DDR = drainRLow | probes.rHigh3Mask;
    // Wait until FET conducts
    if(type & TYPE_N_CHANNEL) {           // N-channel
      // FET conducts when the voltage at drain reaches low level
      while (ADC_PIN & drainAdc);
    } else {                              // P-channel
      // FET conducts when the voltage at drain reaches high level
      while (!(ADC_PIN & drainAdc));
    }
    R_DDR = drainRLow;                     // Set probe-3 to HiZ mode
    // Get voltage of gate
    ADCSRA |= (1 << ADSC);                // Start ADC conversion
    while (ADCSRA & (1 << ADSC));         // Wait until conversion is done
    // Add ADC reading
    if(type & TYPE_N_CHANNEL) {           // N-channel
      vTh += ADCW;                        // vGate = vmeasured
    } else {                              // P-channel
      vTh += (1023 - ADCW);               // vGate = Vcc - vmeasured
    }
  }
  // Calculate vTh
  vTh /= 10;                              // Average of 10 samples
  vTh *= VREF_VCC;                        // Convert to voltage
  vTh /= 1024;                            // Using 10 bit resolution
  // Save data
  fet.vTh = (unsigned int) vTh;
}

// Measure leakage current
unsigned int getLeakageCurrent(void) {
  unsigned int iLeak = 0;                // Return value
  unsigned int vRLow;                      // Voltage at rLow
  unsigned int rShunt;                   // Shunt resistor
  uint32_t value;

//   Setup probes:
//    - use rLow as current shunt
//    - probe-1 = pos / probe-2 = neg / probe-3 = HiZ
//      Diode:    probe-1 = cathode /  probe-2 = anode
//      NPN BJT:  probe-1 = collector / probe-2 = emitter
//      PNP BJT:  probe-1 = emitter / probe-2 = collector

  R_PORT = 0;                             // Set resistor port to Gnd
  R_DDR = probes.rLow2Mask;            // Pull down probe-2 via R low
  ADC_DDR = probes.probe1Adc;           // Set probe-1 to output
  ADC_PORT = probes.probe1Adc;          // Pull-up probe-1 directly
  vRLow = readVoltage5ms(probes.pin2);         // Get voltage at R low

  // Calculate current
  rShunt = parameters.rIntLow + (R_LOW * 10);    // Consider internal resistance of MCU (0.1 Ohms)
  rShunt += 5;                           // For rounding
  rShunt /= 10;                          // Scale to Ohms
  value = vRLow * 100000;                  // Scale to 10nV
  value /= rShunt;                       // in 10nA
  value += 55;                            // For rounding
  value /= 100;                           // Scale to µA
  iLeak = value;

  // Clean up
  setAdcHighZ();                            // Set ADC port to HiZ mode
  setAdcLow();                            // Set ADC port low
  R_DDR = 0;                              // Set resistor port to HiZ mode
  R_PORT = 0;                             // Set resistor port low
  return iLeak;
}

// Check for diode
void checkDiode(void) {
  Diodes *diode;                      // Pointer to diode
  unsigned int v1RLow;                     // Vf #1 with rLow pull-up
  unsigned int v1RHigh;                     // Vf #1 with rHigh pull-up
  unsigned int v1Zero;                   // Vf #1 zero
  unsigned int v2RLow;                     // Vf #2 with rLow pull-up
  unsigned int v2RHigh;                     // Vf #2 with rHigh pull-up
  unsigned int v2Zero;                   // Vf #2 zero

  wdt_reset();                            // Reset watchdog
  dischargeProbes();                      // Try to discharge probes
  if(check.found == COMP_ERROR) return;   // Skip on error

//   DUT could be:
//    - simple diode
//    - protection diode of a MOSFET or another device
//    - intrinsic diode junction of a BJT
//    - small resistor (< 3k)
//    - capacitor (> around 22µF)
// 
//   Solution:
//    - Vf of a diode rises with the current within some limits (about twice
//      for Si and Schottky). Ge, Z-diodes and LEDs are hard to determine.
//      So it might be better to filter out other components.
//    - For a MOSFET pretection diode we have to make sure that the MOSFET
//      in not conducting, to be able to get Vf of the protection diode.
//      So we discharge the gate and run the measurements twice for p and n
//      channel FETs.
//    - Take care about the internal voltage drop of the µC at the cathode
//      for high test currents (rLow).
//    - Filter out resistors by the used voltage divider:
//      k = rLow + rIntHigh + rIntlow
//      vRHigh = vRLow / (k - (k - 1) vRLow / 5V)
//      vRLow = k vRHigh / (1 + (k - 1) vRHigh / 5V)
//    - Filter out caps by checking the voltage before and after measurement
//      with rHigh. In 15ms a 22µF cap would be charged from 0 to 7mV, a larger
//      cap would have a lower voltage. We have to consider that caps also
//      might be charged by EMI.
// 
//    Hints:
//    - rLow drives a current of about 7mA. That's not the best current for
//      measuring Vf. The current for rHigh is about 10.6µA.
//      Most DMMs use 1mA.

  // Vf #1, supporting a possible P-channel MOSFET
  // We assume: probe-1 = A / probe2 = C,
  // set probes: Gnd -- probe-2 / probe-1 -- rLow or rHigh -- Vcc
  setAdcLow();
  ADC_DDR = probes.probe2Adc;           // Pull down cathode directly

  // R_DDR is set to HiZ by dischargeProbes();
  v1Zero = readVoltage(probes.pin1, 1);       // Get voltage at anode

  // Measure voltage across DUT (Vf) with rHigh
  R_DDR = probes.rHigh1Mask;           // Enable rHigh for probe-1
  R_PORT = probes.rHigh1Mask;          // Pull up anode via rHigh

  // Discharge gate
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLUP);
  v1RHigh = readVoltage5ms(probes.pin1);        // Get voltage at anode, ignore voltage at cathode

  // Measure voltage across DUT (Vf) with rLow
  R_DDR = probes.rLow1Mask;            // Enable rLow for probe-1
  R_PORT = probes.rLow1Mask;           // Pull up anode via rLow

  // Discharge gate
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLUP);
  v1RLow = readVoltage5ms(probes.pin1);        // Get voltage at anode
  v1RLow -= readVoltage(probes.pin2, 1);        // Substract voltage at cathode
  dischargeProbes();                      // Try to discharge probes
  if(check.found == COMP_ERROR) return;   // Skip on error

  // Vf #2, supporting a possible N-channel MOSFET
  // We assume: probe-1 = A / probe2 = C,
  // set probes: Gnd -- probe-2 / probe-1 -- rLow or rHigh -- Vcc
  setAdcLow();
  ADC_DDR = probes.probe2Adc;           // Pull down cathode directly
  v2Zero = readVoltage(probes.pin1, 1);       // Get voltage at anode

  // Measure voltage across DUT (Vf) with rHigh
  R_DDR = probes.rHigh1Mask;           // Enable rHigh for probe-1
  R_PORT = probes.rHigh1Mask;          // Pull up anode via rHigh

  // Discharge gate
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLDOWN);
  v2RHigh = readVoltage5ms(probes.pin1);        // Get voltage at anode, ignore voltage at cathode

  // Measure voltage across DUT (Vf) with rLow
  R_DDR = probes.rLow1Mask;            // Enable rLow for probe-1
  R_PORT = probes.rLow1Mask;           // Pull up anode via rLow

  // Discharge gate
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLDOWN);
  v2RLow = readVoltage5ms(probes.pin1);        // Get voltage at anode
  v2RLow -= readVoltage(probes.pin2, 1);        // Substract voltage at cathode
  R_PORT = 0;                             // Stop pulling up

  // Process results, choose between measurements of p and n channel setup
  if(v1RLow > v2RLow) {                     // The higher voltage wins
    v2RLow = v1RLow;
    v2RHigh = v1RHigh;
    v2Zero = v1Zero;
  }

//   vRHigh < 10mV for
//    - resistor < 1k Ohm
//    - very large cap

  if(v2RHigh <= 10) return;                 // Small resistor or very large cap

//   vZero <= 2 for resistor or diode
//   vZero > 2 for cap or diode
//   if vZero > 2 then vRHigh - vZero < 100 for cap
// 
//   Hints:
//    If vZero > 10 and vRHigh is about vZero it's a large cap.
//    As larger the cap as lower vRLow (charging time 15ms).

  v1Zero = v2RHigh - v2Zero;              // Voltage difference
  if((v2Zero > 2) && (v1Zero < 100)) return; // Capacitor

//   The voltages for a resistor will follow the equation:
//     k = rLow + rIntHigh + rIntlow
//     vl = k vRHigh / (1 + (k - 1) vRHigh / 5V)
//   Allow a tolerance of 3%.
//   For vRHigh > 40mV we don't need to check for a resistor.
// 
//   Hint:
//    Actually we could change the thresshold above from 10 t0 40 and
//    remove this test completely. The lowest vRHigh measured for a diode was
//    56mV for a AA118.

  if(v2RHigh < 40) {                        // Resistor (< 3k)
    uint32_t a, b;

    // Calculate expected vRLow based on measured vRHigh in mV, k factor
    b = (R_HIGH * 10) / ((R_LOW * 10) + parameters.rIntHigh + parameters.rIntLow);
    a = b - 1;                            // k - 1
    a /= 5;                               // / 5V
    a *= v2RHigh;                           // *vRHigh
    a += 1000;                            // +1 (1000 for mV)
    b *= 1000;                            // For mV
    b *= v2RHigh;                           // *vRHigh
    b /= a;                               // vRLow in mV

    // Check if calculated vRLow is within some % of measured value
    v1Zero = b;
    v1RLow = v1Zero;
    v1RHigh = v1Zero;
    v1Zero /= 50;                        // 2%
    v1RHigh += v1Zero;                     // 102%
    v1Zero = b;
    v1Zero /= 33;                        // 3%
    v1RLow -= v1Zero;                     // 97% (for resistors near 1k)
    // Resistor
    if((v2RLow >= v1RLow) && (v2RLow <= v1RHigh)) return;
  }
  // If vRLow (Vf) is between 0.15V and 4.64V it's a diode
  if((v2RLow > 150) && (v2RLow < 4640)) {
    // If we haven't found any other component yet
    if(check.found == COMP_NONE || check.found == COMP_RESISTOR) {
      check.found = COMP_DIODE;
    }
    // Save data
    diode = &diodes[check.diodes];
    diode->a = probes.pin1;
    diode->c = probes.pin2;
    diode->vF = v2RLow;                   // Vf for high measurement current
    diode->vF2 = v2RHigh;                  // Vf for low measurement current
    check.diodes++;
  }
}

// Verify MOSFET by checking the body diode
void verifyMosFet(void) {
  byte flag = 0;
  byte n = 0;
  byte anode;
  byte cathode;
  Diodes *diode;                      // Pointer to diode
  // Set expected body diode
  if(check.type & TYPE_N_CHANNEL) {       // N-channel
    anode = fet.s;
    cathode = fet.d;
  } else {                                // P-channel
    anode = fet.d;
    cathode = fet.s;
  }
  diode = &diodes[0];                     // First diode
  // Check all known diodes for reversed one
  while(n < check.diodes) {
    if((diode->a == cathode) && (diode->c == anode)) {
      flag = 1;                           // Signal match
      n = 10;                             // End loop
    }
    n++;                                  // Next diode
    diode++;
  }
  if(flag == 1) {                         // Found reversed diode
    // This can't be a MOSFET, so let's reset
    check.found = COMP_NONE;
    check.type = 0;
    check.done = 0;
  }
}

// Check for BJT or enhancement-mode MOSFET
void checkBjtMosFet(byte bjtType, unsigned int vRLow) {
  byte fetType;                          // MOSFET type
  unsigned int vRCollector;                     // Voltage across collector resistor
  unsigned int vRBase;                     // Voltage across base resistor
  unsigned int bjtLevel;                 // Voltage threshold for BJT
  unsigned int fetLevel;                 // Voltage threshold for FET
  unsigned int iCeo;                     // Leakage current
  unsigned long hFeCommonCollector;                    // hFE (common collector)
  unsigned long hFeCommonEmitter;                    // hFE (common emitter)
  // Init, set probes and measure
  if(bjtType == TYPE_NPN) {              // NPN / n-channel
    bjtLevel = 2557;                     // Voltage across base resistor (5.44µA)
    fetLevel = 3400;                     // Voltage across drain resistor (4.8mA)
    fetType = TYPE_N_CHANNEL;

//     We assume
//      - BJT: probe-1 = C / probe-2 = E / probe-3 = B
//      - FET: probe-1 = D / probe-2 = S / probe-3 = G
//      probes already set to: Gnd -- probe-2 / probe-1 -- rLow -- Vcc
//      drive base/gate via rHigh instead of rLow

    R_DDR = probes.rLow1Mask | probes.rHigh3Mask; // Enable rLow for probe-1 & rHigh for probe-3
    R_PORT = probes.rLow1Mask | probes.rHigh3Mask; // Pull up collector via rLow and base via rHigh
    delay(50);                            // Wait to skip gate charging of a FET
    vRCollector = VREF_VCC - readVoltage(probes.pin1, 1); // vRCollector = Vcc - vNow
    vRBase = VREF_VCC - readVoltage(probes.pin3, 1); // vRBase = Vcc - vb
  } else {                                // PNP / P-channel
    bjtLevel = 977;                      // Voltage across base resistor (2.1µA)
    fetLevel = 2000;                     // Voltage across drain resistor (2.8mA)
    fetType = TYPE_P_CHANNEL;

//     We assume
//      - BJT: probe-1 = E / probe-2 = C / probe-3 = B
//      - FET: probe-1 = S / probe-2 = D / probe-3 = G
//      probes already set to: Gnd -- rLow - probe-2 / probe-1 -- Vcc
//      drive base/gate via rHigh instead of rLow

    R_DDR = probes.rLow2Mask | probes.rHigh3Mask; // Pull down base via rHigh
    vRCollector = readVoltage5ms(probes.pin2);      // vRCollector = vNow
    vRBase = readVoltage(probes.pin3, 1);       // vRBase = vb
  }
  // Distinguish BJT from depletion-mode MOSFET
  if(vRBase > bjtLevel) {                 // vRBase exceeds minimum level of BJT

//     A voltage drop across the base resistor rHigh means that a current
//     is flowing constantly. So this can't be a FET.
// 
//     Problem:
//      A reversed collector and emitter also passes the tests, but with
//      a low hFE. So we need to run two tests to be sure and select the
//      test results with the higher hFE.

    // Two test runs needed at maximium to get right hFE & pins
    if(check.found == COMP_BJT) check.done = 1;
    check.found = COMP_BJT;
    check.type = bjtType;
    // Leakage current
    iCeo = getLeakageCurrent();          // Get leakage current (in µA)

//     Calculate hFE via voltages and known resistors:
//      - hFE = I_c / I_b
//            = (vRCollector / rCollector) / (vRBase / rBase)
//            = (vRCollector * rBase) / (vRBase * rCollector)
//      - consider leakage current:
//        I_c = I_c_conducting - I_c_leak
//            = (vRCollector_conducting / rCollector) - (vRCollector_leak / rCollector)
//            = (vRCollector_conducting - vRCollector_leak) / rCollector
//        -> vRCollector = vRCollector_conducting - vRCollector_leak
//                 = vRCollector_conducting - vRLow

    if(vRCollector > vRLow) vRCollector -= vRLow;       // - vRLow (leakage)
    hFeCommonEmitter = vRCollector * R_HIGH;               // vRCollector * rBase
    hFeCommonEmitter /= vRBase;                       // / vRBase
    hFeCommonEmitter *= 10;                          // Upscale to 0.1
    if(bjtType == TYPE_NPN)              // NPN
      hFeCommonEmitter /= (R_LOW * 10) + parameters.rIntHigh; // / rCollector in 0.1 Ohm
    else                                  // PNP
      hFeCommonEmitter /= (R_LOW * 10) + parameters.rIntLow; // / rCollector in 0.1 Ohm

    // Get hFE for common collector circuit
    hFeCommonCollector = getHFeCommonCollector(bjtType);

    // Keep largest hFE
    if(hFeCommonCollector > hFeCommonEmitter) hFeCommonEmitter = hFeCommonCollector;

    // Only update data if hFE is larger than old one
    if(hFeCommonEmitter > bjt.hFe) {
      // Save data
      bjt.hFe = hFeCommonEmitter;
      bjt.iCeo = iCeo;
      bjt.b = probes.pin3;
      if(bjtType == TYPE_NPN)            // NPN
      {
        bjt.c = probes.pin1;
        bjt.e = probes.pin2;
      } else {                            // PNP
        bjt.c = probes.pin2;
        bjt.e = probes.pin1;
      }
    }
  } else if((vRLow < 97) && (vRCollector > fetLevel)) { // No BJT

//     If there's
//      - just a small leakage current (< 0.1mA) in non-conducting mode
//      - a large vRCollector (= large current) when conducting
//      - a low vRBase (= very low gate current)
//      we got a FET or an IGBT.
//      The drain source channel of a MOSFET is modeled as a resistor
//      while an IGBT acts more like a diode. So we measure the voltage drop
//      across the conducting path. A MOSFET got a low voltage drop based on
//      its R_DS_on and the current. An IGBT got a much higher voltage drop.

    iCeo = readVoltage(probes.pin1, 1) - readVoltage(probes.pin2, 1);
    if(iCeo < 250) {                     // MOSFET
      check.found = COMP_FET;
      check.type = fetType | TYPE_ENHANCEMENT | TYPE_MOSFET;
    } else {                              // IGBT
      check.found = COMP_IGBT;
      check.type = fetType | TYPE_ENHANCEMENT;
    }
    check.done = 1;                       // Transistor found

    // Measure gate threshold voltage
    getGateThreshold(fetType);

    // Save data
    fet.g = probes.pin3;
    if(fetType == TYPE_N_CHANNEL) {      // N-channel
      fet.d = probes.pin1;
      fet.s = probes.pin2;
    } else {                              // P-channel
      fet.d = probes.pin2;
      fet.s = probes.pin1;
    }
  }
}

// Check for a depletion mode FET (self conducting)
void checkDepletionModeFet(unsigned int vRLowLow) {
  unsigned int v1;                       // Voltage #1
  unsigned int v2;                       // Voltage #2

//   Required probe setup (by calling function):
//    - Gnd -- rLow -- probe-2 / probe-1 -- Vcc
// 
//   Check if we got a n-channel JFET or depletion-mode MOSFET
//    - JFETs are depletion-mode only

  if(check.done == 0) {                   // No transistor found yet
    // We assume: probe-1 = D / probe-2 = S / probe-3 = G,
    // probes already set to: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
    R_DDR = probes.rLow2Mask | probes.rHigh3Mask;           // Pull down gate via rHigh
    v1 = readVoltage20ms(probes.pin2);       // Voltage at source
    R_PORT = probes.rHigh3Mask;        // Pull up gate via rHigh
    v2 = readVoltage20ms(probes.pin2);       // Voltage at source

//     If the source voltage is higher when the gate is driven by a positive
//     voltage vs. connected to ground we got a depletion-mode n-channel FET.
//     The source resistor creates a voltage offset based on the current
//     causing V_GS to become negative with the gate pulled down.

    if(v2 > (v1 + 488)) {
      // Compare gate voltages to distinguish JFET from MOSFET
      // Set probes: Gnd -- probe-2 / probe-1 -- rLow -- Vcc
      setAdcLow();                        // Set ADC port to low
      ADC_DDR = probes.probe2Adc;       // Pull down source directly
      R_DDR = probes.rLow1Mask | probes.rHigh3Mask; // Enable rLow for probe-1 & rHigh for probe-3
      R_PORT = probes.rLow1Mask | probes.rHigh3Mask; // Pull up drain via rLow / pull up gate via rHigh
      v2 = readVoltage20ms(probes.pin3);     // Get voltage at gate
      if(v2 > 3911) {                    // MOSFET
        // N-channel depletion-mode MOSFET
        check.type = TYPE_N_CHANNEL | TYPE_DEPLETION | TYPE_MOSFET;
      } else {                            // JFET
        // N-channel JFET (depletion-mode only)
        check.type = TYPE_N_CHANNEL | TYPE_JFET;
      }
      // Save data
      check.found = COMP_FET;
      check.done = 1;
      fet.g = probes.pin3;
      fet.d = probes.pin1;
      fet.s = probes.pin2;
    }
  }
  // Check if we got a p-channel JFET or depletion-mode MOSFET - JFETs are depletion-mode only
  if(check.done == 0) {                   // No transistor found yet
    // We assume: probe-1 = S / probe-2 = D / probe-3 = G,
    // set probes: Gnd -- probe-2 / probe-1 -- rLow -- Vcc
    setAdcLow();                          // Set ADC port to Gnd
    ADC_DDR = probes.probe2Adc;         // Pull down drain directly
    R_DDR = probes.rLow1Mask | probes.rHigh3Mask; // Enable rLow for probe-1 & rHigh for probe-3
    R_PORT = probes.rLow1Mask | probes.rHigh3Mask; // Pull up source via rLow / pull up gate via rHigh
    v1 = readVoltage20ms(probes.pin1);       // Get voltage at source
    R_PORT = probes.rLow1Mask;         // Pull down gate via rHigh
    v2 = readVoltage20ms(probes.pin1);       // Get voltage at source

//     If the source voltage is higher when the gate is driven by a positive
//     voltage vs. connected to ground we got a depletion-mode p-channel FET.
//     The source resistor creates a voltage offset based on the current
//     causing V_GS to become positive with the gate pulled up.

    if(v1 > (v2 + 488)) {
      // Compare gate voltages to distinguish JFET from MOSFET
      // Set probes: probe-2 = HiZ / probe-1 -- Vcc
      ADC_PORT = probes.probe1Adc;      // Pull up source directly
      ADC_DDR = probes.probe1Adc;       // Enable pull up for source
      // Gate is still pulled down via rHigh
      v2 = readVoltage20ms(probes.pin3);     // Get voltage at gate
      if(v2 < 977) {                     // MOSFET
        // P-channel depletion-mode MOSFET
        check.type =  TYPE_P_CHANNEL | TYPE_DEPLETION | TYPE_MOSFET;
      } else {                            // JFET
        // P-channel JFET (depletion-mode only)
        check.type = TYPE_P_CHANNEL | TYPE_DEPLETION | TYPE_JFET;
      }
      // Save data
      check.found = COMP_FET;
      check.done = 1;
      fet.g = probes.pin3;
      fet.d = probes.pin2;
      fet.s = probes.pin1;
    }
  }
}

// Special devices
byte checkThyristorTriac(void) {
  byte flag = 0;                          // Return value
  unsigned int v1;                       // Voltage #1
  unsigned int v2;                       // Voltage #2

//   Check for a thyristor (SCR) or triac
//    - A thyristor conducts also after the gate is discharged as long
//      as the load current stays alive and doesn't reverse polarity.
//    - A triac is a pair of anti-parallel thyristors.
//    - It's possible that the tester doesn't deliver enough current, so
//      it can't detect all types.
// 
//    probes need to be set already to:
//      Gnd -- probe-2 / probe-1 -- rLow -- Vcc

  // We assume: probe-1 = A / probe-2 = C / probe-3 = G, discharge gate
  pullProbe(probes.rLow3Mask, FLAG_10MS | FLAG_PULLDOWN);
  v1 = readVoltage5ms(probes.pin1);          // Get voltage at anode
  R_PORT = 0;                             // Pull down anode
  delay(5);
  R_PORT = probes.rLow1Mask;           // And pull up anode again
  v2 = readVoltage5ms(probes.pin1);          // Get voltage at anode (below rLow)
  // Voltages match behaviour of thyristor or triac
  if((v1 < 1600) && (v2 > 4400)) {
    check.found = COMP_THYRISTOR;         // If not detected as a triac below
    check.done = 1;

//     Check if we got a triac
//      - reverse A and C (A = MT2 / C = MT1)
//      - check if behaviour is the same

    // We assume: probe-1 = MT2 / probe-2 = MT1 / probe-3 = G
    R_DDR = 0;                            // Disable all probe resistors
    R_PORT = 0;
    ADC_PORT = probes.probe2Adc;        // Pull up MT1 directly
    delay(5);
    R_DDR = probes.rLow1Mask;          // Pull down MT2 via rLow
    // Probe-3/gate is in HiZ mode, triac shouldn't conduct without a triggered gate
    v1 = readVoltage5ms(probes.pin1);        // Get voltage at MT2
    // Voltage of MT2 is low (no current)
    if(v1 <= 244) {
      // Trigger gate for reverse direction
      R_DDR = probes.rLow1Mask | probes.rLow3Mask; // And pull down gate via rLow
      v1 = readVoltage5ms(probes.pin3);      // Get voltage at gate
      v2 = readVoltage(probes.pin1, 1);       // Get voltage at MT2
      // Voltage at gate is ok and voltage at MT2 is high (current = triac is conducting)
      if((v1 >= 977) && (v2 >= 733)) {
        // Check if triac still conducts without triggered gate
        R_DDR = probes.rLow1Mask;      // Set probe3 to HiZ mode
        v1 = readVoltage5ms(probes.pin1);    // Get voltage at MT2
        // Voltage at MT2 is still high (current = triac is conducting)
        if(v1 >= 733) {
          // Check if triac stops conducting when load current drops to zero
          R_PORT = probes.rLow1Mask;   // Pull up MT2 via rLow
          delay(5);
          R_PORT = 0;                     // And pull down MT2 via rLow
          v1 = readVoltage5ms(probes.pin1);  // Get voltage at MT2
          // Voltage at MT2 is low (no current = triac is not conducting)
          if(v1 <= 244) {
            // Now we are pretty sure that the DUT is a triac
            check.found = COMP_TRIAC;
          }
        }
      }
    }
    // Save data (we misuse BJT)
    bjt.b = probes.pin3;
    bjt.c = probes.pin1;
    bjt.e = probes.pin2;
    flag = 1;                             // Signal that we found a component
  }
  return flag;
}

// Measure a resistor with low resistance (< 100 Ohms)
unsigned int smallResistor(byte zeroFlag) {
  unsigned int R = 0;                     // Return value
  byte probe;                             // Probe id
  byte mode;                              // Measurement mode
  byte cnt;                           // Sample counter
  unsigned long value;                    // ADC sample value
  unsigned long value1 = 0;               // vRLow temp. value
  unsigned long value2 = 0;               // vR_i_L temp. value
  dischargeProbes();                      // Try to discharge probes
  if(check.found == COMP_ERROR) return R; // Skip on error

//   Measurement method:
//    - use rLow as current shunt
//    - create a pulse and measure voltage at high side of DUT for 1000 times
//    - repeat that for the low side of the DUT

  // Pulse on: GND -- probe 2 / probe 1 -- rLow -- 5V,
  // pulse off: GND -- probe 2 / probe 1 -- rLow -- GND
  setAdcLow();                            // Set ADC port to low
  ADC_DDR = probes.probe2Adc;           // Pull-down probe 2 directly
  R_PORT = 0;                             // Low by default
  R_DDR = probes.rLow1Mask;            // Enable resistor

#define MODE_HIGH B00000001
#define MODE_LOW B00000010

  // Measurement loop
  mode = MODE_HIGH;
  while (mode > 0) {
    // Setup measurement
    if(mode & MODE_HIGH) probe = probes.pin1;
    else probe = probes.pin2;
    wdt_reset();                          // Reset watchdog
    cnt = 0;                          // Reset loop counter
    value = 0;                            // Reset sample value

    // Set ADC to use bandgap reference and run a dummy conversion
    probe |= (1 << REFS0) | (1 << REFS1);
    ADMUX = probe;                        // Set input channel and U reference
    delayMicroseconds(100);               // Time for voltage stabilization
    ADCSRA |= (1 << ADSC);                // Start conversion
    while (ADCSRA & (1 << ADSC));         // Wait until conversion is done

    // Measurement loop (about 1ms per cycle)
    while (cnt < 100) {
      // Create short pulse
      ADC_DDR = probes.probe2Adc;       // Pull-down probe-2 directly
      R_PORT = probes.rLow1Mask;
      // Start ADC conversion, ADC performs S&H after 1.5 ADC cycles (12µs)
      ADCSRA |= (1 << ADSC);              // Start conversion
      // Wait 20µs to allow the ADC to do it's job
      delayMicroseconds(20);
      // Stop pulse
      R_PORT = 0;
      ADC_DDR = probes.probe2Adc | probes.probe1Adc;
      // Get ADC reading (about 100µs)
      while (ADCSRA & (1 << ADSC));       // Wait until conversion is done
      value += ADCW;                      // Add ADC reading
      // Wait
      delayMicroseconds(900);
      cnt++;                          // Next round
    }

    // Convert ADC reading to voltage
    value *= parameters.vBandgap;
    value /= 1024;                        // / 1024 for 10bit ADC
    value /= 10;                          // De-sample to 0.1mV
    // Loop control
    if(mode & MODE_HIGH) {                // Probe #1 / rLow
      mode = MODE_LOW;                    // Switch to low side
      value1 = value;                     // Save measured value
    } else {                              // Probe #2 / R_i_L
      mode = 0;                           // End loop
      value2 = value;                     // Save measured value
    }
  }

  // Process measurement
  if(value1 > value2) {                   // Sanity check
    // I = v/R = (5V - vRLow)/(rLow + R_i_H)
    value = 10UL * VREF_VCC;              // in 0.1 mV
    value -= value1;
    value *= 1000;                        // Scale to µA
    value /= ((R_LOW * 10) + parameters.rIntHigh); // in 0.1 Ohms
    value1 -= value2;                     // in 0.1 mV
    value1 *= 10000;                      // Scale to 0.01 µV
    // R = U/I (including R of probe leads)
    value1 /= value;                      // in 0.01 Ohms
    R = (unsigned int)value1;             // Copy result
    if(zeroFlag == 1) {                   // Auto-zero
      if(R > parameters.rZero) R -= parameters.rZero;
      else R = 0;
    }
  }
#undef MODE_LOW
#undef MODE_HIGH
  // Update vref flag for next ADC run
  parameters.refFlag = (1 << REFS1);          // Set REFS1 bit flag
  return R;
}

// Check for resistor
void checkResistor(void) {
  Resistors *resistor;                // Pointer to resistor
  unsigned long value1;                   // Resistance of measurement #1
  unsigned long value2;                   // Resistance of measurement #2
  unsigned long value;                    // Resistance value
  unsigned long temp;                     // Temp. value
  signed char scale;                      // Resistance scale
  signed char scale2;                     // Resistance scale
  byte n;                                 // Counter
  // Voltages
  unsigned int vRLowHigh;                    // Voltage #1
  unsigned int vRIntLow;                    // Voltage #2
  unsigned int vRLowLow;                    // Voltage #3
  unsigned int vRIntHigh;                    // Voltage #4
  unsigned int vRHighHigh;                    // Voltage #5
  unsigned int vRHighLow;                    // Voltage #6

  wdt_reset();                            // Reset watchdog

//   Resistor measurement
//    - Set up a voltage divider with well known probe resistors and
//      measure the voltage at the DUT.
//    - For low resistance consider the internal resistors of the µC
//      for pulling up/down.
//    - Calculate resistance via the total current and the voltage
//      at the DUT.
//    - We could also use the voltage divider rule:
//      (Ra / Rb) = (va / vb) -> Ra = Rb * (va / vb)
// 
//   check if we got a resistor
//    - A resistor has the same resistance in both directions.
//    - We measure both directions with both probe resistors.

  // We assume: resistor between probe-1 and probe-2,
  //  set probes: Gnd -- probe-2 / probe-1 -- rLow -- Vcc

  setAdcLow();                            // Set ADC port low low
  ADC_DDR = probes.probe2Adc;           // Pull down probe-2 directly
  R_DDR = probes.rLow1Mask;            // Enable rLow for probe-1
  R_PORT = probes.rLow1Mask;           // Pull up probe-1 via rLow
  vRIntLow = readVoltage5ms(probes.pin2);       // Get voltage at internal R of µC
  vRLowHigh = readVoltage(probes.pin1, 1);        // Get voltage at rLow pulled up

//   Check for a capacitor
//    - A capacitor would need some time to discharge.
//    - So we pull down probe-1 via rHigh and measure the voltage.
//    - The voltage will drop immediately for a resistor.

  // Set probes: Gnd -- probe-2 / Gnd -- rHigh -- probe-1
  R_PORT = 0;                             // Set resistor port low
  R_DDR = probes.rHigh1Mask;           // Pull down probe-1 via rHigh
  vRHighLow = readVoltage5ms(probes.pin1);       // Get voltage at probe 1

  // We got a resistor if the voltage is near Gnd
  if(vRHighLow <= 20) {
    // Set probes: Gnd -- probe-2 / probe-1 -- rHigh -- Vcc
    R_PORT = probes.rHigh1Mask;        // Pull up probe-1 via rHigh
    vRHighHigh = readVoltage5ms(probes.pin1);     // Get voltage at rHigh pulled up

    // Set probes: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
    ADC_DDR = probes.probe1Adc;         // Set probe-1 to output
    ADC_PORT = probes.probe1Adc;        // Pull up probe-1 directly
    R_PORT = 0;                           // Set resistor port to low
    R_DDR = probes.rLow2Mask;          // Pull down probe-2 via rLow
    vRIntHigh = readVoltage5ms(probes.pin1);     // Get voltage at internal R of µC
    vRLowLow = readVoltage(probes.pin2, 1);      // Get voltage at rLow pulled down

    // Set probes: Gnd -- rHigh -- probe-2 / probe-1 -- Vcc
    R_DDR = probes.rHigh2Mask;         // Pull down probe-2 via rHigh
    vRHighLow = readVoltage5ms(probes.pin2);     // Get voltage at rHigh pulled down

    // If voltage breakdown is sufficient
    if((vRLowHigh >= 4400) || (vRHighHigh <= 97)) { // R >= 5.1k / R < 9.3k
      if(vRHighHigh < 4972) {                 // R < 83.4M & prevent division by zero

        // Voltage breaks down with low test current and it is not nearly shorted => resistor
        value = 0;                        // Reset value of resistor
        if(vRLowLow < 169) {                // R > 19.5k

          // Use measurements done with rHigh, resistor is less 60MOhm
          if(vRHighLow >= 38) {              // R < 61.4M & prevent division by zero

//             rHigh pulled up (above DUT):
//             I = vRHigh / rHigh = (Vcc - vRHighHigh) / rHigh
//             R = vR / I = vRHighHigh / ((Vcc - vRHighHigh) / rHigh)
//               = rHigh * vRHighHigh / (Vcc - vRHighHigh)
// 
//             Or via voltage divider:
//             R = rHigh * (vdut / vRHigh)
//               = rHigh * (vRHighHigh / (Vcc - vRHighHigh))

            value1 = R_HIGH * vRHighHigh;
            value1 /= (VREF_VCC - vRHighHigh);

//             rHigh pulled down (below DUT):
//             I = vRHighLow / rHigh
//             R = vR / I = (Vcc - vRHighLow) / (vRHighLow / rHigh)
//               = rHigh * (Vcc - vRHighLow) / vRHighLow
// 
//             Or via voltage divider:
//             R = rHigh * (vR / vRHigh)
//               = rHigh * ((Vcc - vRHighLow) / vRHighLow)

            value2 = R_HIGH * (VREF_VCC - vRHighLow);
            value2 /= vRHighLow;

//             Calculate weighted average of both measurements
//              - Voltages below the bandgap reference got a higher resolution
//                (1.1mV instead of 4.9mV).

            if(vRHighHigh < 990) {            // Below bandgap reference
              // Weighted average for vRHighHigh
              value = (value1 * 4);
              value += value2;
              value /= 5;
              value = value2;
            } else if(vRHighLow < 990) {     // Below bandgap reference
              // Weighted average for vRHighLow
              value = (value2 * 4);
              value += value1;
              value /= 5;
            } else {                      // Higher than bandgap reference
              // Classic average
              value = (value1 + value2) / 2;
            }
            value += RH_OFFSET;           // Add offset value for RHigh
            value *= 10;                  // Upscale to 0.1 Ohms
          }
        } else {                          // vRLowLow: R <= 19.5k
          // Use measurements done with rLow
          // Voltages below and above DUT match voltage divider
          // Voltage below DUT can't be higher than above DUT
          if((vRLowHigh >= vRIntLow) && (vRIntHigh >= vRLowLow)) {

//             rLow pulled up (above DUT):
//             I = vRLowRIntHigh / (rLow + rIntHigh) = (Vcc - vRLowHigh) / (rLow + rIntHigh)
//             R = vDut / I
//               = (vRLowHigh - vRIntLow) / ((Vcc - vRLowHigh) / (rLow + rIntHigh))
//               = (rLow + rIntHigh) * (vRLowHigh - vRIntLow) / (Vcc - vRLowHigh)
// 
//             Or via voltage divider:
//             R = (rLow + rIntHigh) * (vR_rIntLow / vRLowRIntHigh) - rIntLow
//               = (rLow + rIntHigh) * (vR_rIntLow / (Vcc - vdut_rIntLow)) - rIntLow

            // Prevent division by zero
            if(vRLowHigh == VREF_VCC) vRLowHigh = VREF_VCC - 1;
            value1 = (R_LOW * 10) + parameters.rIntHigh; // rLow + rIntHigh in 0.1 Ohm
            value1 *= (vRLowHigh - vRIntLow);
            value1 /= (VREF_VCC - vRLowHigh);

//             rLow pulled down (below DUT):
//             I = vRLow_rIntLow / (rLow + rIntLow)
//             R = vR / I
//               = (vRIntHigh - vRLowLow) / (vRLow_rIntLow / (rLow + rIntLow))
//               = (rLow + rIntLow) * (vRIntHigh - vRLowLow) / vRLow_rIntLow
// 
//             Or via voltage divider:
//             R = (rLow + rIntLow) * (vRIntHigh / vRLow_rIntLow) - rIntHigh
//               = (rLow + rIntLow) * ((Vcc - vRLow_rIntLow) / vRLow_rIntLow) - rIntHigh

            value2 = (R_LOW * 10) + parameters.rIntLow; // rLow + rIntLow in 0.1 Ohms
            value2 *= (vRIntHigh - vRLowLow);
            value2 /= vRLowLow;

//             Calculate weighted average of both measurements
//              - Voltages below the bandgap reference got a higher resolution
//                (1.1mV instead of 4.9mV).

            if(vRLowHigh < 990) {            // Below bandgap reference
              // Weighted average for vRHighHigh
              value = (value1 * 4);
              value += value2;
              value /= 5;
            } else if(vRLowLow < 990) {     // Below bandgap reference
              // Weighted average for vRHighLow
              value = (value2 * 4);
              value += value1;
              value /= 5;
            } else {                      // Higher than bandgap reference
              // Classic average
              value = (value1 + value2) / 2;
            }
          } else {                        // May happen for very low resistances
            if(vRLowLow > 4750) value = 1;  // vRLowLow: R < 15 Ohms
            // This will trigger the low resistance measurement below
          }
        }
        // Process results of the resistance measurement
        if(value > 0) {                   // Valid resistor
          scale = -1;                     // 0.1 Ohm by default

          // Meassure small resistor <10 Ohm with special method
          if(value < 100UL) {

            // Run low resistance measurement
            value2 = (unsigned long)smallResistor(1);
            scale2 = -2;                  // 0.01 Ohm

            // Check for valid result
            value1 = value * 2;           // Allow 100% tolerance
            value1 *= 10;                 // Re-scale to 0.01 Ohms
            if(value1 > value2) {         // Got expected value
              value = value2;             // Update data
              scale = scale2;
            }
          }

          // Check for measurement in reversed direction
          n = 0;
          while (n < check.resistors) {   // Loop through resistors
            resistor = &resistors[n];     // Pointer to element
            if((resistor->a == probes.pin1) && (resistor->b == probes.pin2)) {

              // Check if the reversed measurement is within a specific tolerance
              // Set lower and upper tolerance limits
              // < 2 Ohm
              if(cmpValue(value, scale, 2, 0) == -1) {
                temp = value / 2;         // 50%
              } else {                    // >= 2 Ohm
                temp = value / 20;        // 5%
              }
              value1 = value - temp;      // 95% or 50%
              value2 = value + temp;      // 105% or 150%

              // Special case for very low resistance
              // < 0.1 Ohm
              if(cmpValue(value, scale, 1, -1) == -1) {
                value1 = 0;               // 0
                value2 = value * 5;       // 500%
                if(value2 == 0) value2 = 5; // Special case
              }

              // Check if value matches given tolerance
              if((cmpValue(resistor->value, resistor->scale, value1, scale) >= 0) &&
                  (cmpValue(resistor->value, resistor->scale, value2, scale) <= 0)) {
                check.found = COMP_RESISTOR;
                n = 100;                  // End loop and signal match
              } else {                    // No match
                n = 200;                  // End loop and signal mis-match
              }
            } else {                      // No match
              n++;                        // Next one
            }
          }

          // We got a new resistor
          if(n != 100) {                  // Not a known resistor
            if(check.resistors < 3) {     // Prevent array overflow
              // Save data
              // Unused dataset
              resistor = &resistors[check.resistors];
              resistor->a = probes.pin2;
              resistor->b = probes.pin1;
              resistor->value = value;
              resistor->scale = scale;
              check.resistors++;          // Another one found
            }
          }
        }
      }
    }
  }
}

// Compare two scaled values
signed char cmpValue(unsigned long value1, signed char scale1, unsigned long value2, signed char scale2) {
  signed char flag;                       // Return value
  signed char len1, len2;                 // Length

  // Determine virtual length
  len1 = numberOfDigits(value1) + scale1;
  len2 = numberOfDigits(value2) + scale2;
  if((value1 == 0) || (value2 == 0)) {    // Special case
    flag = 10;                            // Perform direct comparison
  } else if(len1 > len2) {                // More digits -> larger
    flag = 1;
  } else if(len1 == len2) {               // Same length
    // Re-scale to longer value
    len1 -= scale1;
    len2 -= scale2;
    while (len1 > len2) {                 // Up-scale value #2
      value2 *= 10;
      len2++;
    }
    while (len2 > len1) {                 // Up-scale value #1
      value1 *= 10;
      len1++;
    }
    flag = 10;                            // Perform direct comparison
  } else {                                // Less digits -> smaller
    flag = -1;
  }

  if(flag == 10) {                        // Perform direct comparison
    if(value1 > value2) flag = 1;
    else if(value1 < value2) flag = -1;
    else flag = 0;
  }
  return flag;
}

// Get number of digits of a value
byte numberOfDigits(unsigned long value) {
  byte cnt = 1;

  while (value >= 10) {
    value /= 10;
    cnt++;
  }
  return cnt;
}

// Measure cap >4.7µF between two probe pins
byte largeCap(Capacitors *cap) {
  byte flag = 3;                          // Return value
  byte tempByte;                          // Temp. value
  byte mode;                              // Measurement mode
  signed char scale;                      // Capacitance scale
  unsigned int tempInt;                   // Temp. value
  unsigned int pulses;                    // Number of charging pulses
  unsigned int vZero;                    // Voltage before charging
  unsigned int vCap;                     // Voltage of DUT
  unsigned int vDrop = 0;                // Voltage drop
  unsigned long raw;                      // Raw capacitance value
  unsigned long value;                    // Corrected capacitance value
  boolean rerun;

  // Setup mode
  mode = FLAG_10MS | FLAG_PULLUP;         // Start with large caps
  do {
    rerun = false;                        // One-Time

//     We charge the DUT with up to 500 pulses each 10ms long until the
//     DUT reaches 300mV. The charging is done via rLow. This method is
//     suitable for large capacitances from 47uF up to 100mF. If we find a
//     lower capacitance we'll switch to 1ms charging pulses and try again
//     (4.7µF up to 47µF).
// 
//     Problem:
//      ReadADC() needs about 5ms (44 runs). We charge the DUT for 10ms and
//      measure for 5ms. During that time the voltage will drop due to
//      resistive losses of the DUT and the measurement itself. So the DUT
//      seems to need more time to reach 300mV causing a higher capacitance
//      be calculated.
// 
//     Remark:
//      The Analog Input Resistance of the ADC is 100MOhm typically.

    // Prepare probes
    dischargeProbes();                    // Try to discharge probes
    if(check.found == COMP_ERROR) return 0; // Skip on error

    // Setup probes: Gnd -- probe 1 / probe 2 -- rLow -- Vcc
    setAdcLow();                          // Set ADC port to low
    ADC_DDR = probes.probe2Adc;         // Pull-down probe 2 directly
    R_PORT = 0;                           // Set resistor port to low
    R_DDR = 0;                            // Set resistor port to HiZ
    vZero = readVoltage(probes.pin1, 1);      // Get zero voltage (noise)

    // Charge DUT with up to 500 pulses until it reaches 300mV
    pulses = 0;
    tempByte = 1;
    while (tempByte) {
      pulses++;
      pullProbe(probes.rLow1Mask, mode); // Charging pulse
      vCap = readVoltage(probes.pin1, 1);     // Get voltage
      vCap -= vZero;                    // Zero offset

      // End loop if charging is too slow
      if((pulses == 126) && (vCap < 75)) tempByte = 0;

      // End loop if 300mV are reached
      if(vCap >= 300) tempByte = 0;

      // End loop if maximum pulses are reached
      if(pulses == 500) tempByte = 0;
      wdt_reset();                        // Reset watchdog
    }

    // If 300mV are not reached DUT isn't a cap or much too large (>100mF)
    // we can ignore that for mid-sized caps
    if(vCap < 300) flag = 1;

    // If 1300mV are reached with one pulse we got a small cap
    if((pulses == 1) && (vCap > 1300)) {
      if(mode & FLAG_10MS) {              // <47µF
        mode = FLAG_1MS | FLAG_PULLUP;    // Set mode (1ms charging pulses)
        rerun = true;                     // And re-run
      } else {                            // <4.7µF
        flag = 2;
      }
    }
  } while (rerun);

//   Check if DUT sustains the charge and get the voltage drop
//    - run the same time as before minus the 10ms charging time
//    - this gives us the approximation of the self-discharging

  if(flag == 3) {
    // Check self-discharging
    tempInt = pulses;
    while (tempInt > 0) {
      tempInt--;                          // Descrease timeout
      vDrop = readVoltage(probes.pin1, 1);    // Get voltage
      vDrop -= vZero;                   // Zero offset
      wdt_reset();                        // Reset watchdog
    }

    // Calculate voltage drop
    if(vCap > vDrop) vDrop = vCap - vDrop;
    else vDrop = 0;

    // If voltage drop is too large consider DUT not to be a cap
    if(vDrop > 100) flag = 0;
  }

//   Calculate capacitance
//    - use factor from pre-calculated largeCapTable
//    - ignore parameters.capZero since it's in the pF range

  if(flag == 3) {
    scale = -9;                           // Factor is scaled to nF

    // Get interpolated factor from table
    raw = getFactor(vCap + vDrop, TABLE_LARGE_CAP);
    raw *= pulses;                        // C = pulses * factor
    if(mode & FLAG_10MS) raw *= 10;       // *10 for 10ms charging pulses
    if(raw > UINT32_MAX / 1000) {         // scale down if C >4.3mF
      raw /= 1000;                        // scale down by 10^3
      scale += 3;                         // Add 3 to the exponent
    }
    value = raw;                          // Copy raw value

    // It seems that we got a systematic error
    value *= 100;
    if(mode & FLAG_10MS) value /= 109;    // -9% for large cap
    else value /= 104;                    // -4% for mid cap

    // Copy data
    cap->a = probes.pin2;                // Pull-down probe pin
    cap->b = probes.pin1;                // Pull-up probe pin
    cap->scale = scale;                   // -9 or -6
    cap->raw = raw;
    cap->value = value;                   // Max. 4.3*10^6nF or 100*10^3µF
  }
  return flag;
}

// Measure cap <4.7µF between two probe pins
byte smallCap(Capacitors *cap) {
  byte flag = 3;                          // Return value
  byte tempByte;                          // Temp. value
  signed char scale;                      // Capacitance scale
  unsigned int ticks;                     // Timer counter
  unsigned int ticks2;                    // Timer overflow counter
  unsigned int vCap;                       // Voltage of capacitor
  unsigned long raw;                      // Raw capacitance value
  unsigned long value;                    // Corrected capacitance value

//   Measurement method used for small caps < 50uF:
//   We need a much better resolution for the time measurement. Therefore we
//   use the µCs internal 16-bit counter and analog comparator. The counter
//   inceases until the comparator detects that the voltage of the DUT is as
//   high as the internal bandgap reference. To support the higher time
//   resolution we use the RHigh probe resistor for charging.
// 
//   Remark:
//   The analog comparator has an Input Leakage Current of -50nA up to 50nA
//   at Vcc/2. The Input Offset is <10mV at Vcc/2.

  ticks2 = 0;                             // Reset timer overflow counter

  // Init hardware, prepare probes
  dischargeProbes();                      // Try to discharge probes
  if(check.found == COMP_ERROR) return 0; // Skip on error

  // Set probes: Gnd -- all probes / Gnd -- RHigh -- probe-1
  R_PORT = 0;                             // Set resistor port to low

  // Set ADC probe pins to output mode
  ADC_DDR = (1 << TP1) | (1 << TP2) | (1 << TP3);
  setAdcLow();                            // Set ADC port to low
  R_DDR = probes.rHigh1Mask;           // Pull-down probe-1 via RHigh

  // Setup analog comparator
  ADCSRB = (1 << ACME);                   // Use ADC multiplexer as negative input
  ACSR =  (1 << ACBG) | (1 << ACIC);      // Use bandgap as positive input, trigger timer1
  ADMUX = (1 << REFS0) | probes.pin1;    // Switch ADC multiplexer to probe 1 and set AREF to Vcc
  ADCSRA = ADC_CLOCK_DIV;                 // Disable ADC, but keep clock dividers
  delayMicroseconds(200);

  // Setup timer
  TCCR1A = 0;                             // Set default mode
  TCCR1B = 0;                             // Set more timer modes

  // Timer stopped, falling edge detection, noise canceler disabled
  TCNT1 = 0;                              // Set Counter1 to 0

  // Clear all flags (input capture, compare A & B, overflow
  TIFR1 = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
  R_PORT = probes.rHigh1Mask;          // Pull-up probe-1 via RHigh

  // Enable timer
  if(check.found == COMP_FET) {
    // Keep all probe pins pulled down but probe-1
    tempByte = (((1 << TP1) | (1 << TP2) | (1 << TP3)) & ~(1 << probes.pin1));
  } else {
    tempByte = probes.probe2Adc;        // Keep just probe-1 pulled down
  }

  // Start timer by setting clock prescaler (1/1 clock divider)
  TCCR1B = (1 << CS10);
  ADC_DDR = tempByte;                     // Start charging DUT

  // Timer loop - run until voltage is reached - detect timer overflows
  while (1) {
     tempByte = TIFR1;                    // Get timer1 flags

     // End loop if input capture flag is set (= same voltage)
     if(tempByte & (1 << ICF1)) break;

     // Detect timer overflow by checking the overflow flag
     if(tempByte & (1 << TOV1)) {
       // Happens at 65.536ms for 1MHz or 8.192ms for 8MHz
       TIFR1 = (1 << TOV1);               // Reset flag
       wdt_reset();                       // Reset watchdog
       ticks2++;                          // Increase overflow counter

       // End loop if charging takes too long (13.1s)
       if(ticks2 == (CPU_FREQ / 5000)) break;
     }
   }

  // Stop counter
  TCCR1B = 0;                             // Stop timer
  TIFR1 = (1 << ICF1);                    // Reset Input Capture flag
  ticks = ICR1;                           // Get counter value

  // Disable charging
  R_DDR = 0;                              // Set resistor port to HiZ mode

  // Catch missed timer overflow
  if((TCNT1 > ticks) && (tempByte & (1 << TOV1))) {
    TIFR1 = (1 << TOV1);                  // Reset overflow flag
    ticks2++;                             // Increase overflow counter
  }

  // Enable ADC again
  ADCSRA = (1 << ADEN) | (1 << ADIF) | ADC_CLOCK_DIV;

  // Get voltage of DUT
  vCap = readVoltage(probes.pin1, 1);           // Get voltage of cap

  // Start discharging DUT
  R_PORT = 0;                             // Pull down probe-2 via RHigh
  R_DDR = probes.rHigh1Mask;           // Enable RHigh for probe-1 again

  // Skip measurement if charging took too long
  if(ticks2 >= (CPU_FREQ / 5000)) flag = 1;

  // Calculate capacitance (<50uF) - use factor from pre-calculated smallCapTable
  if(flag == 3) {
    // Combine both counter values
    raw = (unsigned long)ticks;           // Set lower 16 bits
    raw |= (unsigned long)ticks2 << 16;   // Set upper 16 bits
    if(raw > 2) raw -= 2;                 // Subtract processing time overhead
    scale = -12;                          // Default factor is for pF scale
    if(raw > (UINT32_MAX / 1000)) {       // Prevent overflow (4.3*10^6)
      raw /= 1000;                        // scale down by 10^3
      scale += 3;                         // Add 3 to the exponent (nF)
    }

    // Multiply with factor from table
    raw *= getFactor(parameters.vBandgap + parameters.compOffset, TABLE_SMALL_CAP);

    // Divide by CPU frequency to get the time and multiply with table scale
    raw /= (CPU_FREQ / 10000);
    value = raw;                          // Take raw value

    // Take care about zero offset if feasable
    if(scale == -12) {                    // pF scale
      if(value >= parameters.capZero) {       // If value is larger than offset
        value -= parameters.capZero;          // Substract offset
      } else {                            // If value is smaller than offset
        // We have to prevent a negative value
        value = 0;                        // Set value to 0
      }
    }

    // Copy data
    cap->a = probes.pin2;                // Pull-down probe pin
    cap->b = probes.pin1;                // Pull-up probe pin
    cap->scale = scale;                   // -12 or -9
    cap->raw = raw;
    cap->value = value;                   // Max. 5.1*10^6pF or 125*10^3nF

//     Self-adjust the voltage offset of the analog comparator and internal
//     bandgap reference if C is 100nF up to 20µF. The minimum of 100nF
//     should keep the voltage stable long enough for the measurements.
//     Changed offsets will be used in next test run.

    if(scale == -12 && value >= 100000 || scale == -9 && value <= 20000) {
      signed int              offset;
      signed long             tempLong;

//       We can self-adjust the offset of the internal bandgap reference
//       by measuring a voltage lower than the bandgap reference, one time
//       with the bandgap as reference and a second time with Vcc as
//       reference. The common voltage source is the cap we just measured.

       while (readVoltage(probes.pin1, 1) > 980); // Keep discharging
       R_DDR = 0;                         // Stop discharging
       ticks = readVoltage(probes.pin1, 0);    // vCap with Vcc reference
       ticks2 = readVoltage(probes.pin1, 1);   // vCap with bandgap reference
       R_DDR = probes.rHigh1Mask;      // Resume discharging
       offset = ticks - ticks2;

       // Allow some offset caused by the different voltage resolutions (4.88 vs. 1.07)
       if((offset < -4) || (offset > 4)) { // Offset too large

//          Calculate total offset:
//           - first get offset per mV: Offset / vCap
//           - total offset for vRef: (Offset / vCap) * vRef

         tempLong = offset;
         tempLong *= parameters.vBandgap;    // * vRef
         tempLong /= ticks2;              // / vCap
         parameters.refOffset = (signed char)tempLong;
       }

//       In the cap measurement above the analog comparator compared
//       the voltages of the cap and the bandgap reference. Since the µC
//       has an internal voltage drop for the bandgap reference the
//       µC used actually vbandgap - voffset. We get that offset by
//       comparing the bandgap reference with the voltage of the cap:
//       vCap = vbandgap - voffset -> voffset = vCap - vbandgap

      offset = vCap - parameters.vBandgap;
      // Limit offset to a valid range of -50mV - 50mV
      if((offset > -50) && (offset < 50)) parameters.compOffset = offset;
    }
  }
  return flag;
}

// Measure capacitance between two probe pins
void measureCap(byte probe1, byte probe2, byte id) {
  byte tempByte;                          // Temp. value
  Capacitors *cap;                    // Pointer to cap data structure
  Diodes *diode;                      // Pointer to diode data structure
  Resistors *resistor;                // Pointer to resistor data structure

  // Init - Reset cap data
  cap = &caps[id];
  cap->a = 0;
  cap->b = 0;
  cap->scale = -12;                       // pF by default
  cap->raw = 0;
  cap->value = 0;
  if(check.found == COMP_ERROR) return;   // Skip check on any error

  // Skip resistors - But check for a resistor < 10 Ohm. Might be a large cap.
  if(check.found == COMP_RESISTOR) {
    resistor = &resistors[0];             // Pointer to first resistor
    tempByte = 0;
    while (tempByte < check.resistors) {
      // Got matching pins
      if(((resistor->a == probe1) && (resistor->b == probe2)) ||
          ((resistor->a == probe2) && (resistor->b == probe1))) {

        // Check for low value
        if(cmpValue(resistor->value, resistor->scale, 10UL, 0) == -1)
          tempByte = 99;                  // Signal low resistance and end loop
      }
      tempByte++;                         // Next one
      resistor++;                         // Next one
    }

    // We got a valid resistor
    if(tempByte != 100) return;           // Skip this one
  }

//   Skip measurement for "dangerous" diodes
//    - when Vf collides with the voltage of the capacitance measurement

  diode = &diodes[0];                     // Pointer to first diode
  for (tempByte = 0; tempByte < check.diodes; tempByte++) {
    // Got matching pins and low threshold voltage
    if((diode->c == probe2) && (diode->a == probe1) && (diode->vF < 1500)) return;
    diode++;                              // Next one
  }

  // Run measurements
  updateProbes(probe1, probe2, 0);        // Update bitmasks and probes

  // First run measurement for large caps
  tempByte = largeCap(cap);

  // If cap is too small run measurement for small caps
  if(tempByte == 2) tempByte = smallCap(cap);

  // Check for plausibility
  // If there aren't any diodes in reverse direction which could be detected as capacitors by mistake
  if(check.diodes == 0) {
    // Low resistance might be a large cap
    if(check.found == COMP_RESISTOR) {
      // Report capacitor for large C (> 4.3µF)
      if(cap->scale >= -6) check.found = COMP_CAPACITOR;
    // We consider values below 5pF being just ghosts
    } else if((cap->scale > -12) || (cap->value >= 5UL)) {
      check.found = COMP_CAPACITOR;       // Report capacitor
    }
  }

  // Clean up
  dischargeProbes();                      // Discharge DUT
  // Reset all ports and pins
  setAdcHighZ();                            // Set ADC port to input
  setAdcLow();                            // Set ADC port low
  R_DDR = 0;                              // Set resistor port to input
  R_PORT = 0;                             // Set resistor port low
}

// Measure inductance between two probe pins
byte measureInductance(uint32_t *time, byte mode) {
  byte flag = 3;                          // Return value
  byte test;                              // Test flag
  signed char offset;                     // Counter offet
  unsigned int ticksLow;                   // Timer counter
  unsigned int ticksHigh;                   // Timer overflow counter
  unsigned long cnt;                  // Counter

  // Sanity check
  if(time == NULL) return 0;
  dischargeProbes();                      // Try to discharge probes
  if(check.found == COMP_ERROR) return 0;

//   Measurement modes:
//    - low current: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
//    - high current: Gnd -- probe-2 / probe-1 -- Vcc

//   init hardware

  // Set probes: Gnd -- probe-1 / Gnd -- rLow -- probe-2
  R_PORT = 0;                             // Set resistor port to low
  setAdcLow();                            // Set ADC port to low
  if(mode & MODE_LOW_CURRENT) {           // Low current
    R_DDR = probes.rLow2Mask;          // Pull down probe-2 via rLow
    ADC_DDR = probes.probe1Adc;         // Pull down probe-1 directly
  } else  {                               // High current
    R_DDR = 0;                            // Disable probe resistors

    // Pull down probe-1 and probe-2 directly
    ADC_DDR = probes.probe1Adc | probes.probe2Adc;
  }

  // Setup analog comparator
  ADCSRB = (1 << ACME);                   // Use ADC multiplexer as negative input
  ACSR =  (1 << ACBG) | (1 << ACIC);      // Use bandgap as positive input, trigger timer1
  ADMUX = (1 << REFS0) | probes.pin2;    // Switch ADC multiplexer to probe-2 and set AREF to Vcc
  ADCSRA = ADC_CLOCK_DIV;                 // Disable ADC, but keep clock dividers
  delayMicroseconds(200);                 // Allow bandgap reference to settle

  // Setup timer
  ticksHigh = 0;                            // Reset timer overflow counter
  TCCR1A = 0;                             // Set default mode
  TCCR1B = 0;                             // Set more timer modes

  // Timer stopped, falling edge detection, noise canceler disabled
  TCNT1 = 0;                              // Set Counter1 to 0

  // Clear all flags (input capture, compare A & B, overflow
  TIFR1 = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
  if(mode & MODE_DELAYED_START) {         // Delayed start
    test = (CPU_FREQ / 1000000);          // Cycles per µs

    // Change probes: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
    ADC_PORT = probes.probe1Adc;        // Pull up probe-1 directly

//     Delay timer by about 3-4µs to skip capacitive effects of large inductors
//      - a single loop needs 4 cycles, the last loop run just 3
//      - cycles burnt: <MCU cycles per µs> * 4 - 1

    while (test > 0) {
      test--;
      asm volatile("nop\n\t"::);
    }
    TCCR1B |= (1 << CS10);                // Start timer (1/1 clock divider)
  } else {                                // Immediate start
    TCCR1B |= (1 << CS10);                // Start timer (1/1 clock divider)

    // Change probes: Gnd -- rLow -- probe-2 / probe-1 -- Vcc
    ADC_PORT = probes.probe1Adc;        // Pull up probe-1 directly
  }

  // Timer loop - run until voltage threshold is reached - detect timer overflows
   while (1) {
     test = TIFR1;                        // Get timer1 flags

     // End loop if input capture flag is set (= same voltage)
     if(test & (1 << ICF1)) break;

     // Detect timer overflow by checking the overflow flag
     if(test & (1 << TOV1)) {
       // Happens at 65.536ms for 1MHz or 8.192ms for 8MHz
       TIFR1 = (1 << TOV1);               // Reset flag
       wdt_reset();                       // Reset watchdog
       ticksHigh++;                         // Increase overflow counter

       // If it takes too long (0.26s)
       if(ticksHigh == (CPU_FREQ / 250000)) {
         flag = 0;                        // Signal timeout
         break;                           // End loop
       }
     }
   }

  // Stop counter
  TCCR1B = 0;                             // Stop timer
  TIFR1 = (1 << ICF1);                    // Reset Input Capture flag
  ticksLow = ICR1;                         // Get counter value

  // Prepare cut off: Gnd -- rLow -- probe-2 / probe-1 -- rLow -- Gnd
  R_DDR = probes.rLow2Mask | probes.rLow1Mask;

  // Stop current flow
  setAdcHighZ();

  // Catch missed timer overflow
  if((TCNT1 > ticksLow) && (test & (1 << TOV1))) {
    TIFR1 = (1 << TOV1);                  // Reset overflow flag
    ticksHigh++;                            // Increase overflow counter
  }

  // Enable ADC again
  ADCSRA = (1 << ADEN) | (1 << ADIF) | ADC_CLOCK_DIV;

  // Process counters, combine both counter values
  cnt = (unsigned long) ticksLow;       // Lower 16 bits
  cnt |= (unsigned long) ticksHigh << 16; // Upper 16 bits
  offset = -4;                            // Subtract processing overhead
  if(mode & MODE_DELAYED_START) {         // Delayed start
    // Add MCU cycles for delayed start
    offset += ((CPU_FREQ / 1000000) * 4) - 1;
  } else {                                // Immediate start
    offset -= 1;                          // Timer started one cycle too early
  }
  if(offset >= 0) {                       // Positive offet
    cnt += offset;
  } else {                                // Negative offset
    offset *= -1;                         // Make it positive
    if(cnt < offset) cnt = 0;     // Prevent underflow
    else cnt -= offset;               // Subtract offset
  }
  if(cnt > 0) {
    cnt += (CPU_FREQ / 2000000);      // Add half of cycles for rounding
    cnt /= (CPU_FREQ / 1000000);      // Divide by frequency and scale to µs
  }
  if(cnt <= 1) flag = 2;              // Signal inductance too low
  *time = cnt;                        // Save time
  return flag;
}

// Measure inductance between two probe pins of a resistor
byte measureInductor(Resistors *resistor) {
  byte test = 0;                          // Return value / measurement result
  byte mode;                              // Measurement mode
  byte scale;                             // scale of value
  unsigned int rTotal;                   // Total resistance
  unsigned int factor;                    // Factor
  unsigned long value;                    // value
  unsigned long time1;                    // Time #1
  unsigned long time2;                    // Time #2

  // Reset data
  inductor.scale = 0;
  inductor.value = 0;

  // Sanity check
  if(resistor == NULL) return test;

  // Limit resistor to 2k (feasibilty & prevent variable overflow)
  if(cmpValue(resistor->value, resistor->scale, 2000, 0) >= 0) return test;

//   Manage measurements:
//    - run in immediate and delayed mode to deal with capacitive effects
//      of large inductors and keep smaller time
//    - in case of a small inductance run in high current mode (implies
//      immediate mode only)

  updateProbes(resistor->a, resistor->b, 0); // Update probes
  mode = MODE_LOW_CURRENT;
  test = measureInductance(&time1, mode);
  if(test == 2) {                         // Inductance too low
    // If resistance < 40 Ohms we may run the high current test
    if(cmpValue(resistor->value, resistor->scale, 40, 0) < 0) {
      mode = MODE_HIGH_CURRENT;
      test = measureInductance(&time1, mode);
    }
  } else if(test == 3) {                  // Valid time
    // Let's run the delayed mode
    mode = MODE_LOW_CURRENT | MODE_DELAYED_START;
    test = measureInductance(&time2, mode);
    if(time1 > time2) time1 = time2;      // Lower value wins
  }
  if(test != 3) test = 0;                 // Measurements failed
  // Calculate inductance
  if(test == 3) {
    // Resistances - Total resistance (in 0.1 Ohms) - R_L
    rTotal = RescaleValue(resistor->value, resistor->scale, -1);
    rTotal += parameters.rIntHigh + parameters.rIntLow;

    // Shunt resistance (in 0.1 Ohms)
    factor = parameters.rIntLow;
    if(mode & MODE_LOW_CURRENT) {         // Low current measurement mode
      // Add R_l
      rTotal += (R_LOW * 10);
      factor += (R_LOW * 10);
    }

//     Ratio and factor
//      - ratio = ((vRef * rTotal) / (5V * R_shunt)) * 10^3

    value = parameters.vBandgap + parameters.compOffset;
    value *= rTotal;                     // * rTotal (in 0.1 Ohms)
    value /= factor;                      // / R_shunt (in 0.1 Ohms)
    value /= 5;                           // / 5000mV, * 10^3

    // Get ratio based factor
    factor = getFactor((unsigned int)value, TABLE_INDUCTOR);

//    calculate inductance
//    L = t_stop * rTotal * factor

    scale = -6;                           // µH by default
    value = time1;                        // t_stop
    value *= factor;                      // * factor (µs * 10^-3)
    while (value > 100000) {              // Re-scale to prevent overflow
      value /= 10;
      scale++;
    }
    value *= rTotal;                     // * rTotal (in 0.1 Ohms)
    value /= 10000;
    // Update data
    inductor.scale = scale;
    inductor.value = value;
    // Serial.println("Success");
    test = 1;                             // Signal success
  }
  return test;
}

// Clear single line of display
void lcd_clear_line(unsigned char l) {
  unsigned char c;

  lcd_setcursor(0, l);
  for (c = 0; c < 20; c++) lcd.write(' ');
  lcd_setcursor(0, l);
}

// Write probe pin number to the LCD
void lcd_testpin(unsigned char probe) {
  // Since TP1 is 0 we simply add the value to '1'
  lcd.write('1' + probe);                 // Send data
}

// LINT to here

// Display value and unit
void displayValue(unsigned long value, signed char Exponent, unsigned char Unit) {
  unsigned char prefix = 0;               // Prefix character
  byte offset = 0;                        // Exponent offset to next 10^3 step
  byte index;                             // Index id
  byte Length;                            // String length

  // scale value down to 4 digits
  while (value >= 1000) {
    value += 5;                           // For automagic rounding
    value /= 10;                          // scale down by 10^1
    Exponent++;                           // Increase exponent by 1
  }

  // Determine prefix and offset (= number of digits right of dot)
  if(Exponent >= -12) {                   // Prevent index underflow
    Exponent += 12;                       // Shift exponent to be >= 0
    index = Exponent / 3;                 // Number of 10^3 steps
    offset = Exponent % 3;                // Offset to lower 10^3 step
    if(offset > 0) {                      // Dot required
      index++;                            // Upscale prefix
      offset = 3 - offset;                // Reverse value (1 or 2)
    }
    // Look up prefix in table (also prevent array overflow)
    if(index <= 6) prefix = prefixTable[index];
  }

  // Display value, convert value into string
  utoa((unsigned int)value, outBuffer, 10);
  Length = strlen(outBuffer);

  // We misuse Exponent for the dot position
  Exponent = Length - offset;             // Calculate position
  if(Exponent <= 0) {                     // We have to prepend "0."
    // 0: factor 10 / -1: factor 100
    lcd.write('0');
    lcd.write('.');
    if(Exponent < 0) lcd.write('0');      // Extra 0 for factor 100
  }
  if(offset == 0) Exponent = -1;          // Disable dot if not needed

  // Adjust position to match array or disable dot if set to 0
  Exponent--;

  // Display value and add dot if requested
  index = 0;
  while (index < Length) {                // Loop through string
    lcd.write(outBuffer[index]);          // Display char
    if(index == Exponent) lcd.write('.'); // Display dot
    index++;                              // Next one
  }
  // Display prefix and unit
  if(prefix) lcd.write(prefix);
  if(Unit) lcd.write(Unit);
}

// Display signed value and unit
void displaySignedValue(signed long value, signed char Exponent, unsigned char Unit) {
  // Take care about sign
  if(value < 0) {                         // Negative value
    lcd.write('-');                       // Display: "-"
    value = -value;                       // Make value positive
  }
  // And display unsigned value
  displayValue((signed long)value, Exponent, Unit);
}

// Tell user to create or remove short-circuit of all three probes
void ShortCircuit(byte mode) {
  byte Run = 0;                           // Loop control
  byte test;                              // Test feedback
  byte d = 0;
  test = allProbesShorted();              // Get current status
  if(mode == 0) {                         // Remove short
    // Some shorted
    if(test != 0) d = 1;
  } else {                                // Create short
    // Some unshorted
    if(test != 3) d = 1;
  }
  // If required tell user what to do
  if(d) {
    lcd_clear();
    if(mode == 0) {                       // Remove short
      lcd.print(F("Remove"));
    } else {                              // Create short
      lcd.print(F("Create"));
    }
    lcd_setcursor(0, 2);
    lcd.print(F("Short Circuit"));        // Display: short circuit!
    Run = 1;                              // Enter loop
  }
  // Wait until all probes are dis/connected
  while (Run == 1) {
    test = allProbesShorted();            // Check for short circuits
    if(mode == 0) {                       // Remove short
      if(test == 0) Run = 0;              // End loop if all removed
    } else {                              // Create short
      if(test == 3) Run = 0;              // End loop if all shorted
    }
    if(Run == 1)                          // If not done yet
      delay(50);                          // Wait a little bit
    else                                  // If done
      delay(200);                         // Time to debounce
  }
}

// Detect keypress of test push button
byte testKey() {
  byte i;

  if(gogohut) {
    if(digitalRead(TEST_BUTTON) == LOW) {
      lcd_clear();
      lcd.print(F("Leaving continuous"));
      while(digitalRead(TEST_BUTTON) == LOW);
      delay(1000);
      gogohut = 0;
      return 0;
    } else {
      delay(500);
      return 1;
    }
  }
  for(;;) {
    i = testButton.timeup();
    if(!i) {
      delay(10);
      continue;
    }
    if(i >= SEC5_PRESS) return 2;
    else if(i >= SEC1_PRESS) gogohut = 1;
    return 1;
  }
}

// Show failed test
void showFail(void) {
  // Display info
  lcd.print(F("No or O/C device"));                 // Display: No component
}

// Show Error                             //Only for Standalone Version!
void showError() {
  if(check.type == TYPE_DISCHARGE)        // Discharge failed
  {
    lcd.print("Battery?");             // Display: Battery?

    // Display probe number and remaining voltage
    lcd_setcursor(0, 2);
    lcd_testpin(check.probe);
    lcd.write(':');
    lcd.write(' ');
    displayValue(check.v, -3, 'V');
  }
}

// Display Uf of a diode
void showDiode_Uf(Diodes *diode) {
  // Sanity check
  if(diode == NULL) return;

  // Display Vf
  displayValue(diode->vF, -3, 'V');
}

// Display capacitance of a diode
void showDiode_C(Diodes *diode) {
  // Sanity check
  if(diode == NULL) return;

  // Get capacitance (opposite of flow direction)
  measureCap(diode->c, diode->a, 0);

  // And show capacitance
  displayValue(caps[0].value, caps[0].scale, 'F');
}

// Show diode
void showDiode(void) {
  Diodes *D1;                         // Pointer to diode #1
  Diodes *D2 = NULL;                  // Pointer to diode #2
  byte SkipFlag = 0;                      // Flag for anti-parallel diodes
  byte A = 5;                             // id of common anode
  byte C = 5;                             // id of common cothode
  unsigned int iLeak;                    // Leakage current

  D1 = &diodes[0];                        // Pointer to first diode

  // Figure out which diodes to display
  if(check.diodes == 1) {                 // Single diode
    C = D1->c;                            // Make anode first pin
  } else {                                // To many diodes
    D1 = NULL;                            // Don't display any diode
    showFail();                           // And tell user
    return;
  }
  lcd_createChar(5, symDIODE1);
  lcd_createChar(6, symDIODE2);
  lcd_createChar(7, symF);
  lcd_createChar(2, symLOW1);
  lcd_createChar(3, symLOW2);
  lcd_createChar(4, symR);
  lcd_createChar(0, symV);
  lcd_createChar(1, symI);
  lcd_clear();
  lcd.print(F("Diode"));
  lcd_setcursor(19, 0);
  lcd_testpin(D1->a);                     // Display pin #1
  lcd_setcursor(19, 1);
  lcd.write(5);
  lcd_setcursor(19, 2);
  lcd.write(6);
  lcd_setcursor(19, 3);
  lcd_testpin(D1->c);                     // Display pin #2
  lcd_setcursor(0, 1);                    // Move to line #1


//   display:
//    - Uf (forward voltage)
//    - reverse leakage current
//    - capacitance

  // Uf
  lcd.write((byte) 0);
  lcd.write((byte) 7);
  lcd.print(F("= "));
  showDiode_Uf(D1);                       // First diode
  lcd_setcursor(0, 2);
  // Display low current Uf
  lcd.write((byte) 0);
  lcd.write((byte) 7);
  lcd.write((byte) 2);
  lcd.write((byte) 3);
  lcd.print(F(" = "));
  displayValue(D1->vF2, -3, 'V');

  // Reverse leakage current
  updateProbes(D1->c, D1->a, 0);          // Reverse diode
  iLeak = getLeakageCurrent();           // Get current (in µA)
  lcd_setcursor(0, 3);
  lcd.write((byte) 1);
  lcd.write((byte) 4);
  lcd.print(F("= "));
  displayValue(iLeak, -6, 'A');          // Display current

  // Capacitance
  lcd.write(' ');
  lcd.print(F("C = "));
  showDiode_C(D1);                        // First diode
}

// Show BJT
void showBjt(void) {
  Diodes *diode;                      // Pointer to diode
  char *String;                           // Display string pointer
  byte cnt;                           // Counter
  byte aPin;                             // Pin acting as anode
  byte cPin;                             // Pin acting as cathode
  byte e, c;
  long int V_BE;                          // V_BE
  signed int Slope;                       // Slope of forward voltage

  wdt_disable();                          // Disable watchdog
  // Display type
  if(check.type == TYPE_NPN) lcd_createChar(2, symNPN);
  else lcd_createChar(2, symPNP);
  lcd_clear();
  lcd_home();
  if(check.type == TYPE_NPN) {            // NPN
    e = 3;
    c = 1;
    lcd.print(F("Trans. Bipolar   NPN"));
  } else {                                // PNP
    e = 1;
    c = 3;
    lcd.print(F("Trans. Bipolar   PNP"));
  }
  lcd_createChar(7, symFE);
  lcd_createChar(5, symBE1);
  lcd_createChar(6, symBE2);
  lcd_createChar(3, symCE1);
  lcd_createChar(4, symCE2);
  lcd_createChar(0, symH);
  lcd_createChar(1, symV);
  if(check.type == TYPE_NPN) lcd_createChar(2, symNPN);
  else lcd_createChar(2, symPNP);
  lcd_setcursor(19, e);
  lcd_testpin(bjt.e);                     // Display emitter pin
  lcd_setcursor(14, 2);
  lcd_testpin(bjt.b);                     // Display base pin
  lcd_setcursor(19, c);
  lcd_testpin(bjt.c);                     // Display collector pin
  lcd_setcursor(16, 2);
  lcd.write('-');
  lcd.write((byte) 2);
  lcd_setcursor(0, 1);                    // Move to line #1
  lcd.write((byte) 0);
  lcd.write((byte) 7);
  lcd.print(F("  = "));
  displayValue(bjt.hFe, 0, 0);
  // Display V_BE (taken from diode forward voltage)
  diode = &diodes[0];                     // Get pointer of first diode
  cnt = 0;
  while (cnt < check.diodes) {        // Check all diodes
   // Set pins based on BJT type
    if(check.type == TYPE_NPN) {
      // diode B -> E
      aPin = bjt.b;
      cPin = bjt.e;
    } else {
      // diode E -> B
      aPin = bjt.e;
      cPin = bjt.b;
    }
    // If the diode matches the transistor
    if(diode->a == aPin && diode->c == cPin) {
  lcd_setcursor(0, 2);                    // Move to line #2
  lcd.write(1);
  lcd.write((byte) 5);
  lcd.write((byte) 6);
  lcd.print(F(" = "));

//       Vf is quite linear for a logarithmicly scaled I_b.
//       So we may interpolate the Vf values of low and high test current
//       measurements for a virtual test current. Low test current is 10µA
//       and high test current is 7mA. That's a logarithmic scale of
//       3 decades.

      // Calculate slope for one decade
      Slope = diode->vF - diode->vF2;
      Slope /= 3;
      // Select V_BE based on hFE
      if(bjt.hFe < 100) {                 // Low hFE

//         BJTs with low hFE are power transistors and need a large I_b
//         to drive the load. So we simply take Vf of the high test current
//         measurement (7mA).

        V_BE = diode->vF;
      } else if(bjt.hFe < 250) {          // Mid-range hFE

//         BJTs with a mid-range hFE are signal transistors and need
//         a small I_b to drive the load. So we interpolate Vf for
//         a virtual test current of about 1mA.

        V_BE = diode->vF - Slope;
      } else {                            // High hFE

//         BJTs with a high hFE are small signal transistors and need
//         only a very small I_b to drive the load. So we interpolate Vf
//         for a virtual test current of about 0.1mA.

        V_BE = diode->vF2 + Slope;
      }
    if(check.type == TYPE_PNP) V_BE = - V_BE;
      displaySignedValue(V_BE, -3, 'V');
      // I_CEO: collector emitter cutoff current (leakage)
      lcd_setcursor(0, 3);
      lcd.write('I');
      lcd.write((byte) 3);
      lcd.write((byte) 4);
      lcd.print(F(" = "));                   // Display: iCeo=
      displayValue(bjt.iCeo, -6, 'A');   // Display current
      cnt = check.diodes;             // End loop
    } else {
      cnt++;                          // Increase counter
      diode++;                            // Next one
    }
  }
  wdt_reset();
}

// Show MOSFET/IGBT extras
void showFetIGBT_Extras(byte Symbol) {
  // Instrinsic diode
  lcd_clear();
  if(check.diodes > 0) {
    lcd.write(' ');                       // Display space
    lcd.write(Symbol);                    // Display diode symbol
  }

  // Gate threshold voltage
  lcd_setcursor(0, 1);
  lcd.print(F("Vth = "));
  displayValue(fet.vTh, -3, 'V');        // Display vTh in mV
  lcd_setcursor(0, 2);
  // Display gate capacitance
  lcd.print(F("Cgs = "));                 // Display: Cgs=
  measureCap(fet.g, fet.s, 0);            // Measure capacitance
  // Display value and unit
  displayValue(caps[0].value, caps[0].scale, 'F');
}

// Show FET
void showFet(void) {
  byte data;                              // Temp. data
  byte Symbol;                            // Intrinsic diode

  // Set variables based on channel mode
  if(check.type & TYPE_N_CHANNEL)         // n-channel
    data = 'N';
  else                                    // p-channel
    data = 'P';
  // Display type
  if(check.type & TYPE_MOSFET) {          // MOSFET
    lcd.print(F("MOS"));
    lcd.print(F("FET"));
  // Display channel type
    lcd_setcursor(0, 1);
    lcd.write(data);                      // Display: N / P
    lcd.print(F("-channel"));
  } else  {                               // JFET symJFET
    if(data == 'P')
      lcd_createChar(4, symPFetG);
    else
      lcd_createChar(4, symNFetG);
    lcd_createChar(5, symFet);
    lcd_createChar(6, symJFetT);
    lcd_createChar(7, symJFetB);
    if(check.type == TYPE_NPN) lcd_createChar(2, symNPN);
    else lcd_createChar(2, symPNP);
    lcd_setcursor(16, 2);
    lcd.write(4);
    lcd.write(5);
    // Display pins
    lcd_setcursor(17, 1);
    lcd.write(6);
    lcd.print(F(" ?"));
    lcd_setcursor(14, 2);
    lcd_testpin(fet.g);                   // Display gate pin
    lcd_setcursor(17, 3);
    lcd.write(7);
    lcd.print(F(" ?"));
    lcd_setcursor(0, 0);
    lcd.print(F("Tran. JFET "));
    lcd.write(data);
    lcd.print(F("-channel"));
    lcd_setcursor(0, 1);
    lcd.print(F("Cannot"));
    lcd_setcursor(0, 2);
    lcd.print(F("identify S/D"));
    lcd_setcursor(0, 3);
    lcd.print(F("for a JFET"));
    lcd_setcursor(2, 3);
    lcd.print(F("r"));
    return;
  }
  // Display mode
  if(check.type & TYPE_MOSFET) {          // MOSFET
    lcd_setcursor(0, 1);
    if(check.type & TYPE_ENHANCEMENT)     // Enhancement mode
      lcd.print(F("Enhancement"));
    else                                  // Depletion mode
      lcd.print(F("Depletion"));
  }
  // Pins
  lcd_setcursor(0, 2);                    // Move to line #2
  lcd.print(F("GDS = "));
  lcd_testpin(fet.g);                     // Display gate pin
  lcd_testpin(fet.d);                     // Display drain pin
  lcd_testpin(fet.s);                     // Display source pin

  // Extra data for MOSFET in enhancement mode
  if(check.type & (TYPE_ENHANCEMENT | TYPE_MOSFET)) {
    // Show diode, vTh and Cgs
    showFetIGBT_Extras(Symbol);
  }
}

// Show IGBT
void showIGBT(void) {
  byte data;                              // Temp. data
  byte Symbol;                            // Intrinsic diode
  // Set variables based on channel mode
  if(check.type & TYPE_N_CHANNEL)         // n-channel
    data = 'N';
  else                                    // p-channel
    data = 'P';
  lcd.print(F("IGBT"));                    // Display: IGBT
  // Display channel type
  lcd.write(' ');
  lcd.write(data);                        // Display: N / P
  lcd.print(F("-channel"));
  // Display mode
  lcd_setcursor(0, 1);
  if(check.type & TYPE_ENHANCEMENT)     // Enhancement mode
    lcd.print(F("Enhancement"));
  else                                  // Depletion mode
    lcd.print(F("Depletion"));
  // Pins
  lcd_setcursor(0, 2);                    // Move to line #2
  lcd.print(F("GCE = "));
  lcd_testpin(fet.g);                     // Display gate pin
  lcd_testpin(fet.d);                     // Display collector pin
  lcd_testpin(fet.s);                     // Display emitter pin
  // Show diode, vTh and C_CE
  showFetIGBT_Extras(Symbol);
}

// Show special components like Thyristor and Triac
void showSpecial(void) {
  // Display component type
  if(check.found == COMP_THYRISTOR) {
    lcd.print(F("Thyristor"));
  } else if(check.found == COMP_TRIAC) {
    lcd.print(F("Triac"));                 // Display: triac
  }
  // Display pins
  lcd_setcursor(0, 2);                    // Move to line #2
  lcd.print(F("GAC = "));                     // Display: GAK
  lcd_testpin(bjt.b);                     // Display gate pin
  lcd_testpin(bjt.c);                     // Display anode pin
  lcd_testpin(bjt.e);                     // Display cathode pin
}

// Show resistor
void showResistor(void) {
  Resistors *R1;                      // Pointer to resistor #1
  byte Pin;                               // id of common pin
  R1 = &resistors[0];                     // Pointer to first resistor

  // Get inductance and display if relevant
  lcd_createChar(5, symRES1);
  lcd_createChar(6, symRES2);
  if(measureInductor(R1) == 1) {
    // Display the pins, first resistor
    lcd_createChar(5, symIND1);
    lcd_createChar(6, symIND2);
    lcd_clear();
    lcd.print(F("Inductor"));
    lcd_setcursor(19, 0);
    lcd_testpin(R1->a);
    lcd_setcursor(19, 1);
    lcd.write(5);
    lcd_setcursor(19, 2);
    lcd.write(6);
    lcd_setcursor(19, 3);
    lcd_testpin(R1->b);
    lcd_setcursor(0, 1);                  // Move to line #2
    lcd.print(F("Value: "));
    displayValue(inductor.value, inductor.scale, 'H');
    lcd_setcursor(0, 2);                  // Move to line #2
    lcd.print(F("Res.: "));
      displayValue(R1->value, R1->scale, LCD_CHAR_OMEGA);
  } else {
  // Display the pins, first resistor
    lcd_clear();
    lcd.print(F("Resistor"));
    lcd_setcursor(2, 0);
    lcd.write('s');
    lcd_setcursor(19, 0);
    lcd_testpin(R1->a);
    lcd_setcursor(19, 1);
    lcd.write(5);
    lcd_setcursor(19, 2);
    lcd.write(6);
    lcd_setcursor(19, 3);
    lcd_testpin(R1->b);
    lcd_setcursor(0, 1);                  // Move to line #2
    lcd.print(F("Value: "));
    displayValue(R1->value, R1->scale, LCD_CHAR_OMEGA);
  }
}

// Show capacitor
void showCapacitor(void) {
  Capacitors *maxCap;                 // Pointer to largest cap
  Capacitors *cap;                    // Pointer to cap
  byte cnt;                           // Loop counter

  // Find largest cap
  maxCap = &caps[0];                      // Pointer to first cap
  cap = maxCap;
  for (cnt = 1; cnt <= 2; cnt++)
  {
    cap++;                                // Next cap
    if(cmpValue(cap->value, cap->scale, maxCap->value, maxCap->scale) == 1)
    {
      maxCap = cap;
    }
  }
  // Serial.println(maxCap->value);
  // Serial.println(maxCap->scale);

  // Display pinout
  lcd_createChar(5, symCAP1);
  lcd_createChar(6, symCAP2);
  lcd_clear();
  lcd.print(F("Capacitor"));
  lcd_setcursor(19, 0);
  lcd_testpin(maxCap->a);                 // Display pin #1
  lcd_setcursor(19, 1);
  lcd.write(5);
  lcd_setcursor(19, 2);
  lcd.write(6);
  lcd_setcursor(19, 3);
  lcd_testpin(maxCap->b);                 // Display pin #2
  lcd_setcursor(0, 1);                    // Move to line #2
  lcd.print(F("Value: "));
  // And show capacitance
  // Serial.println(maxCap->scale);
  displayValue(maxCap->value, maxCap->scale, 'F');
}

// Load adjustment values
void loadAdjust(void) {
  if(EEPROM.read(PARAM_MAGIC) == ROM_MAGIC) {
    ReadEEP();
  } else {
    // Default Values
    parameters.rIntLow = R_MCU_LOW;
    parameters.rIntHigh = R_MCU_HIGH;
    parameters.rZero = R_ZERO;
    parameters.capZero = C_ZERO;
    parameters.refOffset = VREF_OFFSET;
    parameters.compOffset = COMPARATOR_OFFSET;
    SaveEEP();
  }
}

// Selftest
byte SelfTest(void) {
  byte flag = 0;                          // Return value
  byte test = 1;                          // Test counter
  byte cnt;                           // Loop counter
  byte displayFlag;                       // Display flag
  unsigned int value0;                      // Voltage/value
  // Voltages/values
  signed int value1 = 0, value2 = 0, value3 = 0;

  ShortCircuit(1);                        // Make sure all probes are shorted
  // Loop through all tests
  while (test <= 6) {
    cnt = 1;
    // Repeat each test 5 times
    while (cnt <= 5) {
      // Display test number
      lcd_clear();
      lcd.write('T');                     // Display: T
      lcd.write('0' + test);              // Display test number
      lcd.write(' ');
      displayFlag = 1;                    // Display values by default
      // Tests
      switch (test) {
        case 1:                           // Reference voltage
          value0 = readVoltage(0x0e, 1);          // Dummy read for bandgap stabilization
          value0 = readVoltage(0x0e, 1);          // Read bandgap reference voltage
          lcd.print(F("Vref"));
          lcd_setcursor(0, 2);
          displayValue(value0, -3, 'V');    // Display voltage in mV
          displayFlag = 0;                // Reset flag
          break;

        case 2:                           // Compare rLow resistors (probes still shorted)
          lcd.print(F("+rLow-"));
          lcd.write(' ');
          lcd.print(F("12 13 23"));
          // Set up a voltage divider with the rLow's, substract theoretical voltage of voltage divider
          // TP1: Gnd -- rLow -- probe-2 -- probe-1 -- rLow -- Vcc
          R_PORT = 1 << (TP1 * 2);
          R_DDR = (1 << (TP1 * 2)) | (1 << (TP2 * 2));
          value1 = readVoltage20ms(TP3);
          value1 -= ((long)VREF_VCC * (R_MCU_LOW + R_LOW)) / (R_MCU_LOW + R_LOW + R_LOW + R_MCU_HIGH);
          // TP1: Gnd -- rLow -- probe-3 -- probe-1 -- rLow -- Vcc
          R_DDR = (1 << (TP1 * 2)) | (1 << (TP3 * 2));
          value2 = readVoltage20ms(TP2);
          value2 -= ((long)VREF_VCC * (R_MCU_LOW + R_LOW)) / (R_MCU_LOW + R_LOW + R_LOW + R_MCU_HIGH);
          // TP1: Gnd -- rLow -- probe-3 -- probe-2 -- rLow -- Vcc
          R_PORT = 1 << (TP2 * 2);
          R_DDR = (1 << (TP2 * 2)) | (1 << (TP3 * 2));
          value3 = readVoltage20ms(TP2);
          value3 -= ((long)VREF_VCC * (R_MCU_LOW + R_LOW)) / (R_MCU_LOW + R_LOW + R_LOW + R_MCU_HIGH);
          break;

        case 3:                           // Compare RHigh resistors (probes still shorted)
          lcd.print(F("+RHigh-"));
          lcd.write(' ');
          lcd.print(F("12 13 23"));
          // Set up a voltage divider with the RHigh's
          // TP1: Gnd -- RHigh -- probe-2 -- probe-1 -- RHigh -- Vcc
          R_PORT = 2 << (TP1 * 2);
          R_DDR = (2 << (TP1 * 2)) | (2 << (TP2 * 2));
          value1 = readVoltage20ms(TP3);
          value1 -= (VREF_VCC / 2);
          // TP1: Gnd -- RHigh -- probe-3 -- probe-1 -- RHigh -- Vcc
          R_DDR = (2 << (TP1 * 2)) | (2 << (TP3 * 2));
          value2 = readVoltage20ms(TP2);
          value2 -= (VREF_VCC / 2);
          // TP1: Gnd -- RHigh -- probe-3 -- probe-2 -- RHigh -- Vcc
          R_PORT = 2 << (TP2 * 2);
          R_DDR = (2 << (TP2 * 2)) | (2 << (TP3 * 2));
          value3 = readVoltage20ms(TP1);
          value3 -= (VREF_VCC / 2);
          break;

        case 4:                           // Un-short probes
          ShortCircuit(0);                // Make sure probes are not shorted
          cnt = 100;                  // Skip test
          displayFlag = 0;                // Reset flag
          break;

        case 5:                           // RHigh resistors pulled down
          lcd.print(F("RHigh-"));
          // TP1: Gnd -- RHigh -- probe
          R_PORT = 0;
          R_DDR = 2 << (TP1 * 2);
          value1 = readVoltage20ms(TP1);
          // TP1: Gnd -- RHigh -- probe
          R_DDR = 2 << (TP2 * 2);
          value2 = readVoltage20ms(TP2);
          // TP1: Gnd -- RHigh -- probe
          R_DDR = 2 << (TP3 * 2);
          value3 = readVoltage20ms(TP3);
          break;

        case 6:                           // RHigh resistors pulled up
          lcd.print(F("RHigh+"));
          // TP1: probe -- RHigh -- Vcc
          R_DDR = 2 << (TP1 * 2);
          R_PORT = 2 << (TP1 * 2);
          value1 = readVoltage20ms(TP1);
          // TP1: probe -- RHigh -- Vcc
          R_DDR = 2 << (TP2 * 2);
          R_PORT = 2 << (TP2 * 2);
          value2 = readVoltage20ms(TP2);
          // TP1: probe -- RHigh -- Vcc
          R_DDR = 2 << (TP3 * 2);
          R_PORT = 2 << (TP3 * 2);
          value3 = readVoltage20ms(TP3);
          break;
      }

      // Reset ports to defaults
      R_DDR = 0;                          // Input mode
      R_PORT = 0;                         // All pins low

      // Display voltages/values of all probes
      if(displayFlag) {
        lcd_setcursor(0, 2);              // Move to line #2
        displaySignedValue(value1, 0 , 0);  // Display TP1
        lcd.write(' ');
        displaySignedValue(value2, 0 , 0);  // Display TP2
        lcd.write(' ');
        displaySignedValue(value3, 0 , 0);  // Display TP3
      }
      // Wait and check test push button
      if(cnt < 100) {                 // When we don't skip this test
          displayFlag = testKey();        // Catch key press or timeout
        // Short press -> next test / long press -> end selftest
        if(displayFlag > 0) {
          cnt = 100;                  // Skip current test anyway
          if(displayFlag == 2) test = 100; // Also skip selftest
        }
      }
      cnt++;                          // Next run
    }
    test++;                               // Next one
  }
  flag = 1;                               // Signal success
  return flag;
}

// Self adjustment
byte selfAdjust(void) {
  byte flag = 0;                          // Return value
  byte test = 1;                          // Test counter
  byte cnt;                           // Loop counter
  byte displayFlag;                       // Display flag

  // Voltages
  unsigned int value1 = 0, value2 = 0, value3 = 0;
  byte capCount = 0;                    // Number of C_Zero measurements
  unsigned int capSum = 0;                // Sum of C_Zero values
  byte rCount = 0;                      // Number of R_Zero measurements
  unsigned int RSum = 0;                  // Sum of R_Zero values
  byte rIntLowCount = 0;                   // Number of vrIntLow measurements
  unsigned int vrIntLow = 0;                 // Sum of vrIntLow values
  byte rIntHighCount = 0;                   // Number of vRInternalHighl measurements
  unsigned int vRInternalHighl = 0;                 // Sum of vrIntLow values
  unsigned long value0;                     // Temp. value

  // Measurements
  ShortCircuit(1);                        // Make sure all probes are shorted
  while (test <= 5) {
    cnt = 1;

    // Repeat each measurement 5 times
    while (cnt <= 5) {
      // Display test number
      lcd_clear();
      lcd.write('A');                     // Display: a
      lcd.write('0' + test);              // Display number
      lcd.write(' ');
      displayFlag = 1;                    // Display values by default

      // Tests
      switch (test) {
        case 1:                           // Resistance of probe leads (probes shorted)
          lcd.print(F("R0"));
          lcd.write(' ');
          lcd.print(F("12 13 23"));
           // The resistance is for two probes in series and we expect it to be
           // smaller than 1.00 Ohms, i.e. 0.50 Ohms for a single probe
          updateProbes(TP2, TP1, 0);
          value1 = smallResistor(0);
          if(value1 < 100) {                // Within limit
            RSum += value1;
            rCount++;
          }
          updateProbes(TP3, TP1, 0);
          value2 = smallResistor(0);
          if(value2 < 100) {                // Whithin limit
            RSum += value2;
            rCount++;
          }
          updateProbes(TP3, TP2, 0);
          value3 = smallResistor(0);
          if(value3 < 100) {                // Within limit
            RSum += value3;
            rCount++;
          }
          break;

        case 2:                           // Un-short probes
          ShortCircuit(0);                // Make sure probes are not shorted
          cnt = 100;                  // Skip test
          displayFlag = 0;                // Reset display flag
          break;

        case 3:                           // Internal resistance of µC in pull-down mode
          lcd.print(F("Ri-"));

          // TP1:  Gnd -- Ri -- probe -- rLow -- Ri -- Vcc
          setAdcLow();
          ADC_DDR = 1 << TP1;
          R_PORT = 1 << (TP1 * 2);
          R_DDR = 1 << (TP1 * 2);
          value1 = readVoltage5ms(TP1);
          vrIntLow += value1;

          // TP2: Gnd -- Ri -- probe -- rLow -- Ri -- Vcc
          ADC_DDR = 1 << TP2;
          R_PORT =  1 << (TP2 * 2);
          R_DDR = 1 << (TP2 * 2);
          value2 = readVoltage5ms(TP2);
          vrIntLow += value2;

          // TP3: Gnd -- Ri -- probe -- rLow -- Ri -- Vcc
          ADC_DDR = 1 << TP3;
          R_PORT =  1 << (TP3 * 2);
          R_DDR = 1 << (TP3 * 2);
          value3 = readVoltage5ms(TP3);
          vrIntLow += value3;
          rIntLowCount += 3;
          break;

        case 4:                           // Internal resistance of µC in pull-up mode
          lcd.print(F("Ri+"));

          // TP1: Gnd -- Ri -- rLow -- probe -- Ri -- Vcc
          R_PORT = 0;
          ADC_PORT = 1 << TP1;
          ADC_DDR = 1 << TP1;
          R_DDR = 1 << (TP1 * 2);
          value1 = VREF_VCC - readVoltage5ms(TP1);
          vRInternalHighl += value1;

          // TP2: Gnd -- Ri -- rLow -- probe -- Ri -- Vcc
          ADC_PORT = 1 << TP2;
          ADC_DDR = 1 << TP2;
          R_DDR = 1 << (TP2 * 2);
          value2 = VREF_VCC - readVoltage5ms(TP2);
          vRInternalHighl += value2;

          // TP3: Gnd -- Ri -- rLow -- probe -- Ri -- Vcc
          ADC_PORT = 1 << TP3;
          ADC_DDR = 1 << TP3;
          R_DDR = 1 << (TP3 * 2);
          value3 = VREF_VCC - readVoltage5ms(TP3);
          vRInternalHighl += value3;
          rIntHighCount += 3;
          break;

        case 5:                           // Capacitance offset (PCB and probe leads)
          lcd.print(F("C0"));
          lcd.write(' ');
          lcd.print(F("12 13 23"));

          // The capacitance is for two probes and we expect it to be less than 100pF.
          measureCap(TP2, TP1, 0);
          value1 = (unsigned int)caps[0].raw;

          // Limit offset to 100pF
          if((caps[0].scale == -12) && (caps[0].raw <= 100)) {
            capSum += value1;
            capCount++;
          }
          measureCap(TP3, TP1, 1);
          value2 = (unsigned int)caps[1].raw;

          // Limit offset to 100pF
          if((caps[1].scale == -12) && (caps[1].raw <= 100)) {
            capSum += value2;
            capCount++;
          }
          measureCap(TP3, TP2, 2);
          value3 = (unsigned int)caps[2].raw;

          // Limit offset to 100pF
          if((caps[2].scale == -12) && (caps[2].raw <= 100)) {
            capSum += value3;
            capCount++;
          }
          break;
      }

      // Reset ports to defaults
      setAdcHighZ();                        // Input mode
      setAdcLow();                        // All pins low
      R_DDR = 0;                          // Input mode
      R_PORT = 0;                         // All pins low
      // Display values
      if(displayFlag) {
        lcd_setcursor(0, 2);              // Move to line #2
        displayValue(value1, 0 , 0);        // Display TP1
        lcd.write(' ');
        displayValue(value2, 0 , 0);        // Display TP2
        lcd.write(' ');
        displayValue(value3, 0 , 0);        // Display TP3
      }

      // Wait and check test push button
      if(cnt < 100) {                 // When we don't skip this test
          displayFlag = testKey();        // Catch key press or timeout

        // Short press -> next test / long press -> end selftest
        if(displayFlag > 0) {
          cnt = 100;                  // Skip current test anyway
          if(displayFlag == 2) test = 100; // Also skip selftest
        }
      }
      cnt++;                          // Next run
    }
    test++;                               // Next one
  }

  // Calculate values and offsets
  // Capacitance auto-zero: calculate average value for all probe pairs
  if(capCount == 15) {
    // Calculate average offset (pF)
    parameters.capZero = capSum / capCount;
    flag++;
  }

  // Resistance auto-zero: calculate average value for all probes pairs
  if(rCount == 15) {
    // Calculate average offset (0.01 Ohms)
    parameters.rZero = RSum / rCount;
    flag++;
  }

  // rIntLow & rIntHigh
  if((rIntLowCount == 15) && (rIntHighCount == 15)) {

//     Calculate rIntLow and rIntHigh using the voltage divider rule:
//     Ri = rLow * (vRi / vRLow)
//      - scale up by 100, round up/down and scale down by 10

    // Use values multiplied by 3 to increase accuracy
    vrIntLow /= 5;                           // Average sum of 3 vrIntLow
    vRInternalHighl /= 5;                           // Average sum of 3 vRInternalHighl
    value1 = (VREF_VCC * 3) - vrIntLow - vRInternalHighl; // vRLow * 3

    // rIntLow, rLow * vRi / vRLow in 0.01 Ohm
    value0 = ((unsigned long) R_LOW * 100 * vrIntLow) / value1;
    value0 += 5;                            // For automagic rounding
    value0 /= 10;                           // Scale down to 0.1 Ohm
    if(value0 < 250UL) {                    // < 25 Ohms
      parameters.rIntLow = (unsigned int) value0;
      flag++;
    }

    // rIntHigh, rLow * vRi / vRLow in 0.01 Ohm
    value0 = ((unsigned long) R_LOW * 100 * vRInternalHighl) / value1;
    value0 += 5;                            // For automagic rounding
    value0 /= 10;                           // Scale down to 0.1 Ohm
    if(value0 < 280UL) {                    // < 29 Ohms
      parameters.rIntHigh = (unsigned int) value0;
      flag++;
    }
  }

  // Show values and offsets
  showAdjust();
  if(flag == 4) flag = 1;                 // All adjustments done -> success
  else flag = 0;                          // Signal error
  return flag;
}

// Show adjustment values and offsets
void showAdjust(void) {
  // Display rIntLow and rIntHigh
  lcd_clear();
  lcd.print(F("Ri-"));
  lcd.write(' ');
  displayValue(parameters.rIntLow, -1, LCD_CHAR_OMEGA);
  lcd_setcursor(0, 2);
  lcd.print(F("Ri+"));
  lcd.write(' ');
  displayValue(parameters.rIntHigh, -1, LCD_CHAR_OMEGA);
  testKey();                              // Let the user read

  // Display C-Zero
  lcd_clear();
  lcd.print(F("C0"));
  lcd.write(' ');
  displayValue(parameters.capZero, -12, 'F'); // Display C0 offset

  // Display R-Zero
  lcd_setcursor(0, 2);
  lcd.print(F("R0"));
  lcd.write(' ');
  displayValue(parameters.rZero, -2, LCD_CHAR_OMEGA); // Display R0
  testKey();                              // Let the user read

  // Display offset of bandgap reference
  lcd_clear();
  lcd.print(F("Vref"));
  lcd.write(' ');
  displaySignedValue(parameters.refOffset, -3, 'V');

  // Display offset of analog comparator
  lcd_setcursor(0, 2);
  lcd.print(F("AComp"));
  lcd.write(' ');
  displaySignedValue(parameters.compOffset, -3, 'V');
  testKey();                              // Let the user read
}

// Update values stored in EEPROM
void SaveEEP(void) {
  // Ri of µC in low mode
  EEPROM.updateInt(PARAM_RIL, parameters.rIntLow);
  // Ri of µC in low mode
  EEPROM.updateInt(PARAM_RIL, parameters.rIntHigh);
  // Resistance of probe leads
  EEPROM.updateInt(PARAM_RZERO, parameters.rZero);
  // Capacitance offset: PCB + wiring + probe leads
  EEPROM.update(PARAM_RZERO, parameters.capZero);
  // Voltage offset of bandgap reference
  EEPROM.update(PARAM_REFOFFSET, parameters.refOffset);
  // Voltage offset of analog comparator
  EEPROM.update(PARAM_COMPOFFSET, parameters.compOffset);
  EEPROM.update(PARAM_MAGIC, ROM_MAGIC);
}

// Read values stored in EEPROM
void ReadEEP(void) {
  parameters.rIntLow = EEPROM.readInt(PARAM_RIL);
  parameters.rIntHigh = EEPROM.readInt(PARAM_RIH);
  parameters.rZero = EEPROM.readInt(PARAM_RZERO);
  parameters.capZero = EEPROM.read(PARAM_CAPZERO);
  parameters.refOffset = EEPROM.read(PARAM_REFOFFSET);
  parameters.compOffset = EEPROM.read(PARAM_COMPOFFSET);
}

void adjustAndSave(void) {
  lcd_clear();
  lcd.print(F("Adjust"));
  lcd_setcursor(0, 1);
  lcd.print(F(" - press button"));
  testKey();
  lcd_setcursor(0, 2);
  lcd.print(F("Adjusting..."));
  selfAdjust();
  lcd_setcursor(0, 1);
  lcd.print(F(" - press button"));
  testKey();
  lcd_clear();
  lcd.print(F("Adjust Done"));
  lcd_setcursor(0, 1);
  lcd.print(F(" - press button"));
  testKey();
  lcd_clear();
  lcd.print(F("Save"));
  lcd_setcursor(0, 1);
  lcd.print(F(" - press button"));
  SaveEEP();
  lcd_clear();
  lcd.print(F("Done"));
  lcd_setcursor(0, 1);
  lcd.print(F(" - press button"));
  testKey();
  return;
}
