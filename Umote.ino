/*
Umote - Universal IR remote control
===================================

Umote is a universal remote control for TV or other devices that use IR for their 
remote operation. The software captures the IR codes from an existing remote for 
the device and associates it with a switch. When the key is pressed the programmed 
code is transmitted.

The device has 3 modes - Program, Clear and Run mode. Run mode is the default
if the others are not enabled.

Program Mode
============
Program mode is used to record and associate the output from an existing remote to a
switch on Umote. Setup mode is started by pressing one of the PROG_PIN or CLEAR_PIN 
switches during power up. Umote will simutaneously flash all the LEDs to show that it 
is entering program mode, at which point the PROG_PIN switch should be released.

To program any switch on Umote, press the key. It will start flashing. Point the 
existing remote at Umote and activate the required function. Umote will capture 
and ecode the code being sent and store it. If the capture is successful the switch 
stops flashing and turns off. If the dxata stream could not be decoded, it will be 
stored as raw data, the LED flashes rapidly a few times and the process may not 
have succeeded. Leave Program mode by resetting Umote (power down and back up again).

Clear Mode
==========
Clear mode is used to clear programmed switches on Umote. Clear mode is started by 
pressing the PROG_PIN and CLEAR_PIN switches during power up. Umote will simultaneously
double flash all the LEDs to show that it is entering program mode, at which
point the switches should be released.

To clear a key, press the switch to be cleared. The key will flash twice and the 
programmed function is cleared. Leave Clear mode by resetting Umote (power down and 
back up again).

Run Mode
========
Run mode is the default operation when the device is powered up. The remote will flash 
each LED independently in turn to indicate that run mode is enabled. In this mode, 
pressing any key will transmit the code programmed in setup mode. The switch LED will 
remain on for the duration of the transmission plus a WAIT_DELAY to prevent. Once the 
LED has switched off another switch may be pressed.

Arduino Pin Requirements
========================
+---------------------+-----------------------+-------------+
| Description         | Defined               | Uno default |
+---------------------+-----------------------+-------------+
| IR Receiver         | IR_RECV_IRQ & IRLib   | IRQ0, pin 2 |
| IR Receiver Power   | IR_RECV_PWR           | 4           |
| IR Transmitter      | IR_XMIT_PIN & IRLib   | 3           |
| Switch 0..5         | S[] array initialiser | A0..A5      |
| LED 0..5            | S[] array initialiser | 12..7       |
| Program Mode Sel    | PROG_PIN              | an S[] pin  |
| Clear Mode Sel      | CLEAR_PIN             | an S[] pin  |
+---------------------+-----------------------+-------------+

Dependencies & References
=========================
RTLib - Library for sending and receiving IR codes https://github.com/cyborg5/IRLib/ with
additional documentation at http://tech.cyborg5.com/irlib/.

Version
=======
v1.0 November 2015 
- Initial Release

v1.1 December 2015 
- Fixed button feedback when not configured
- Fixed EEPROM overflow/wraparound
- Cleaned up type definitions

Copyright
=========
Copyright(C) 2015 Marco Colli.All rights reserved.

This is free software; you can redistribute it and / or modify it under the terms of the 
GNU Lesser General Public License as published by the Free Software Foundation; either
version 2.1 of the License, or(at your option) any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.
*/ 
#include <IRLib.h>
#include <EEPROM.h>
#include <avr/power.h>
#include <avr/sleep.h>

// Code modifiers definitions -------------------
//#define UMOTE_DBG   // uncomment this line to enable debug output
#define USE_SLEEP   // uncomment this line to sleep the processor in run mode (conserve battery power)

#ifdef UMOTE_DBG
#define DEBUGS(s)     { Serial.print(F(s)); }
#define DEBUG(s, v)   { Serial.print(F(s)); Serial.print(v); }
#define DEBUGX(s, v)  { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); }
#else
#define DEBUGS(s)
#define DEBUG(s, v)
#define DEBUGX(s, v)
#endif // UMOTE_DBG

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

// Type and enum definitions --------------------
typedef enum runMode_t    { MODE_RUN, MODE_CLR, MODE_PROG };      // Run Modes
typedef enum switchState_t  { SW_IDLE, SW_DEBOUNCE };             // Switch detection FSM states
typedef enum clrState_t   { CS_INIT, CS_IDLE, CS_CLEAR, CS_END }; // Clear States for FSM 
typedef enum progState_t  { SS_INIT, SS_IDLE, SS_RECV, SS_DECODED, SS_RECORDED, SS_FLASH, SS_END }; // Setup States for FSM
typedef enum runState_t   { RS_INIT, RS_SLEEP, RS_IDLE, RS_SEND, RS_WAIT1, RS_WAIT2, RS_END };      // Run States for FSM

// Data encapsulating what we need to know about an IR message
typedef struct irCode_t
{
  IRTYPES   type;         // type of remote code identified
  uint32_t  value[2];     // code values for the remote (generally value & bits)

  uint8_t   dataLen;      // number of valid bytes in data
  uint16_t  data[RAWBUF/2]; // raw data stored for the device. RAWBUF too big for EEPROM size.
};

// Data required to keep track of a switch
typedef struct switch_t
{
  uint8_t   pinSignal;    // signal pin - switch
  uint8_t   pinFeedback;  // feedback pin - LED 

  switchState_t state;    // last input fsm state for this switch
  uint32_t  timeLast;     // for timed operations or debouncing switch
};

// Constants driving program behaviour ----------
const uint8_t NO_PIN = 0xff;    // no pin defined for this input or output
const uint8_t NO_INPUT = 0xff;  // no input pin has been activated
const uint8_t SWITCH_ACTIVE = LOW;  // the switch active state (ie, pressed)
const uint8_t LED_ON = HIGH;    // LED on state
const uint8_t LED_OFF = LOW;    // LED off state

#define PROG_PIN  (S[0].pinSignal)  // input pin used for program mode selection on power up
#define CLEAR_PIN (S[3].pinSignal)  // input pin used for clear program mode selection on power up
const uint8_t IR_RECV_PWR = 4;  // IR receiver power on/off pin
const uint8_t IR_RECV_IRQ = 0;  // IR receiver pin 2, use interrupt 0 on UNO
const uint8_t IR_XMIT_PIN = 3;  // IR transmitter pin

const uint16_t  FLASH_PERIOD = 500;   // total time taken for a flash sequence (ON/OFF), in milliseconds
const uint8_t   FLASH_OK = 2;         // number of short flashes if OK
const uint8_t   FLASH_ERR = 4;        // Number of short flashes if ERROR

const uint16_t  WAIT_DELAY = 1000;    // wait delay on switch sending, in milliseconds
const uint8_t   DEBOUNCE_DELAY = 20;  // switch debounce delay, in milliseconds
const uint8_t   SEND_REPEATS = 3;     // number of times to send an IR message
const uint16_t  SEND_DELAY = 90;      // delay between repeats in microseconds

const uint8_t IRSENDKHZ = 38;   // Default sending frequency in KHz
const uint16_t EEPROM_OFFSET = 0;  // Offset from EEPROM base address for data

// Global variables -----------------------------
IRrecvPCI irRecv(IR_RECV_IRQ);  // IRLib object for receiving message
IRsend    irSend;               // IRLib object for sending message
IRdecode  irResults;            // IRLib object to decode messages
irCode_t  irMesg;               // Umote data for an IR message

runMode_t runMode;              // Umote execution mode

// Data table for the Umote switches. Define as many as are required
// Placemarkers can use the NO_PIN definition to avoid function
static switch_t S[] = 
{
  { A0, 12, SW_IDLE, 0 },
  { A1, 11, SW_IDLE, 0 },
  { A2, 10, SW_IDLE, 0 },
  { A3,  9, SW_IDLE, 0 },
  { A4,  8, SW_IDLE, 0 },
  { A5,  7, SW_IDLE, 0 },
};

// create constant for number of remote control buttons 
const uint8_t MAX_SWITCH = ARRAY_SIZE(S);

#ifdef USE_SLEEP
// Interrupt Handling for I/O pins --------------
// If this is enabled (default) then Umote will sleep while waiting for
// a key press in Run mode. This minimises the power requirements and 
// extends battery life.

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Use one Routine to handle each group
ISR(PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
}

ISR(PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
}

ISR(PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
}

void sleepNow(void)
// Low power sleep mode - all unnecessary functions are shut down and 
// the processor is put in max power saving mode. Code execution is
// suspended in this function and a level change IRQ from the switches 
// being activated will wake up the processor and exit the function.
{
  DEBUGS("\nSleeping ...");
  delay(100);	// Allow Serial to finish up if it is used

  // turn off all unnecessary functions
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
 
  // Choose sleep mode and go to sleep
  cli();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  
  // Upon waking up, execution continues from this point.
  sleep_disable();
  power_timer0_enable();
  power_timer1_enable();
  power_timer2_enable();
  DEBUGS("\nAwake!");
}
// ----------------------------------------------
#endif  // USE_SLEEP

// EEPROM Save and retrieve routines ------------
// Data programmed for each switch is stored in EEPROM, in consecutive memory
// locations starting at address 0. The entire t_irCode structure is stored
// in the EEPROM as a unit (ie, all at once), offset by its index in the S[]
// data array.
// Debug will print each transaction in and out of EEPROM.

void eepromWrite(uint8_t idx, struct irCode_t *c)
// Write the current IR code as the idx'th element 
// of the data table. Each entry in the data table consist of t_irCode 
// records in consecutive memory locations.
{
  uint16_t  addr = (idx * sizeof(irCode_t)) + EEPROM_OFFSET;

  DEBUG("\nSW", idx);
  DEBUG(": @", addr);
  DEBUG(", type ", (uint8_t)c->type);
  if (c->type != UNKNOWN)
  {
    DEBUGX(" code ", c->value[0]);
  }
  else
  {
    DEBUGS(" data ");
    for (uint8_t i = 0; i < c->dataLen; i++)
    {
      DEBUG("", c->data[i]);
      DEBUGS(",");
    }
  }

  EEPROM.put(addr, *c);
}

void eepromRead(uint8_t idx, irCode_t *c)
// Read the current IR code data from the idx'th element 
// of the data table. See eepromWrite() for data table format.
{
  uint16_t  addr = (idx * sizeof(irCode_t)) + EEPROM_OFFSET;

  DEBUG("\nSW", idx);
  DEBUG(": @", addr);

  EEPROM.get(addr, *c);

  DEBUG(", type ", (uint8_t)c->type);
  if (c->type != UNKNOWN)
  {
    DEBUGX(" code ", c->value[0]);
    c->dataLen = 0;
    memset((void *)c->data, 0, RAWBUF);
  }
  else
  {
    DEBUG(" data (", c->dataLen);
    DEBUGS(") ");
    for (uint8_t i = 0; i < c->dataLen; i++)
    {
      DEBUG("", c->data[i]);
      DEBUGS(", ");
    }
  }
}
// ----------------------------------------------

// Switch handling code -------------------------
// Mechanical switches used for Umote need to be debounced. Data in the 
// S[] array is used to read I/O and keep track of millis() timing used to 
// debounce the switch and the current state of the FSM. All Switches are 
// pulled high during initialisation, so the activated state will be LOW.

boolean handleInputSignal(uint8_t sw)
// return true when a transition from inactive to active switch 
// states has been detected.
{
  bool b;
  
  // is the pin configured?
  if (S[sw].pinSignal == NO_PIN)
    return(false);

  // read the current state and run through the FSM
  b = (digitalRead(S[sw].pinSignal) == SWITCH_ACTIVE);

  switch (S[sw].state)
  {
  case SW_IDLE:		
    // Waiting for first transition
    if (b)
    {
      S[sw].state = SW_DEBOUNCE;		// edge detected, initiate debounce
      S[sw].timeLast = millis();
      b = false;
    }
    break;

  case SW_DEBOUNCE:
    // Wait for debounce time to run out and ignore any key switching
    // that might be going on as this may be bounce
    if ((millis() - S[sw].timeLast) < DEBOUNCE_DELAY)
    {
      b = false;
      break;
    }

    // after debounce, return the actual current state of this key and reset the
    // state to the initial state
    // fall through ...

  default:  // should never see default label, but this will reset just in case
    S[sw].state = SW_IDLE;
    break;
  }

  return(b);
}

uint8_t checkInputSignals(void)
// Iterate through all the input switches to see if one has been activated
{
  for (uint8_t i = 0; i < MAX_SWITCH; i++)
    if (handleInputSignal(i)) return(i);

  return(NO_INPUT);
}
// ----------------------------------------------

// Feedback LED handling code -------------------
// LEDs are used extensively to provide feedback to the user. Each switch input 
// has an associated LED - the intent is that the switch has indepent LED associated
// that can provide visual feedback. Definitions for the LED pins are part of the
// S[] array.

boolean getFlasher(uint8_t sel)
// Return true if the LED for the specified switch is on.
{
  if (S[sel].pinFeedback == NO_PIN) return(false);

  return(digitalRead(S[sel].pinFeedback) == LED_ON);
}

void setFlasher(uint8_t ledState, uint8_t sel = NO_PIN)
// Set the feedback LED for the selection
// If the selection is NO_PIN then set all the feedback LEDs
{
  if (sel == NO_PIN)
  {
    for (uint8_t i = 0; i < MAX_SWITCH; i++)
      if (S[i].pinFeedback != NO_PIN) digitalWrite(S[i].pinFeedback, ledState);
  }
  else
    digitalWrite(S[sel].pinFeedback, ledState);
}

void doFlasher(uint8_t count, uint32_t time, uint8_t sel = NO_PIN)
// Flash the feedback LEDs for the specified delay period with repeats
// This is BLOCKING and will only return when the flash sequence is complete.
{
  for (uint8_t i = 0; i < count; i++)
  {
    setFlasher(LED_ON, sel);
    delay(time);
    setFlasher(LED_OFF, sel);
    delay(time);
  }
}

void runFlasher(uint8_t sel)
// Flash the feedback LED for the specified switch with period FLASH_PERIOD
// This is NON-BLOCKING and needs to be called repeatedly to complete the 
// flash sequence. Useful when we need to indicate that the next action will 
// affect a switch whilst still check other things related to the switch.
{
  // Flash the display with non-blocking code
  if (millis() - S[sel].timeLast < FLASH_PERIOD)
    return;

  S[sel].timeLast = millis(); // set up for the next run

  // toggle the feedback LED
  setFlasher(getFlasher(sel) ? LED_OFF : LED_ON, sel);
}
// ----------------------------------------------

void modeClr(void)
// Code runs when the remote is in clear mode.
// See comments at the file header for how this mode works.
{
  static clrState_t state = CS_INIT;  // FSM state variable
  static uint8_t  curSelection = 0;    // once we have a selection, remember it
#ifdef UMOTE_DBG
  static boolean  bInIdle = false;      // blocking sentinel to avoid reams of debug output
#endif

  switch (state)
  {
  case CS_INIT:  // Show the UI that we are in clear mode
    DEBUGS("\nCS_INIT");
    state = CS_IDLE;
    break;

  case CS_IDLE:  // Wait for a key to be pressed
#ifdef UMOTE_DBG
    if (!bInIdle) DEBUGS("\nCS_IDLE");
    bInIdle = true;
#endif
    // If one of the switches has been pressed, process that switch
    if ((curSelection = checkInputSignals()) != NO_INPUT)
    {
      state = CS_CLEAR;
#ifdef UMOTE_DBG
      bInIdle = false;
#endif
      DEBUG("\nClearing sw", curSelection);
    }
    break;

  case CS_CLEAR:  // Clear out the key
    DEBUGS("\nCS_CLEAR");
    irMesg.type = UNKNOWN;
    irMesg.dataLen = 0;
    eepromWrite(curSelection, &irMesg);
    doFlasher(FLASH_OK, FLASH_PERIOD / FLASH_OK, curSelection);
    state = CS_END;
    break;

  case CS_END:
    DEBUGS("\nCS_END");
  default:
    state = CS_IDLE;
  }
}

void modeProg(void)
// Code runs when the remote is in program mode.
// See comments at the file header for how this mode works.
{
  static progState_t state = SS_INIT;  // FSM state variable
  static uint8_t  curSelection = 0;    // once we have a selection, remember it
  static uint8_t  flashCount = 0;      // end of sequence flash count
#ifdef UMOTE_DBG
  static boolean  bInIdle = false;      // blocking sentinel to avoid reams of debug output
#endif

  switch (state)
  {
  case SS_INIT:  // Show the UI that we are in program mode
    DEBUGS("\nSS_INIT");
    // Receiver is only used in programming mode, so turn it on
    if (IR_RECV_PWR != NO_PIN)
    {
      pinMode(IR_RECV_PWR, OUTPUT);
      digitalWrite(IR_RECV_PWR, HIGH);
    }
    state = SS_IDLE;
    break;

  case SS_IDLE:  // waiting for user to tell us which button will be set up
#ifdef UMOTE_DBG
    if (!bInIdle) DEBUGS("\nSS_IDLE");
    bInIdle = true;
#endif
    // If one of the switches has been pressed, process that switch
    if ((curSelection = checkInputSignals()) != NO_INPUT)
    {
      state = SS_RECV;
      S[curSelection].timeLast = millis(); 
      irRecv.enableIRIn();      // start the IR receiver
#ifdef UMOTE_DBG
      bInIdle = false;
#endif
      DEBUG("\nSetting up sw", curSelection);
    }
    break;

  case SS_RECV:  // now receiving/learning what this button will transmit
    DEBUGS("\nSS_RECV");
    runFlasher(curSelection);     // flashing LED
    if (irRecv.GetResults(&irResults)) 
    {
      irResults.IgnoreHeader = true;
      irResults.decode();
#ifdef UMOTE_DBG
      DEBUGS("\nReceived data: ");
      irResults.DumpResults();
#endif // UMOTE_DBG
      if (irResults.decode_type == UNKNOWN)
        state = SS_RECORDED;
      else
        state = SS_DECODED;
    }
    break;

  case SS_DECODED:  // got the decoded data - save it
    DEBUGS("\nSS_DECODED");
    // copy out relevant fields
    irMesg.type = irResults.decode_type;
    irMesg.value[0] = irResults.value;
    irMesg.value[1] = irResults.bits;
    irMesg.dataLen = 0;
    eepromWrite(curSelection, &irMesg);

    // Set up for final flashing
    flashCount = FLASH_OK;
    state = SS_FLASH;
    break;
  
  case SS_RECORDED:  // error in decoding, we will save the raw buffer (and hope for the best!)
    DEBUGS("\nSS_RECORDED");
    // copy out relevant fields
    irMesg.type = irResults.decode_type;
    irMesg.value[0] = irResults.value;
    irMesg.value[1] = irResults.bits;
    if (irResults.rawlen > ARRAY_SIZE(irMesg.data))
    {
      DEBUGS("\nRaw data bigger than buffer - truncating");
      irResults.rawlen = ARRAY_SIZE(irMesg.data);
    }
    irMesg.dataLen = irResults.rawlen;
    for (uint8_t i = 0; i<irMesg.dataLen; i++)
      irMesg.data[i] = irResults.rawbuf[i];
    eepromWrite(curSelection, &irMesg);

    // Set up for final flashing
    flashCount = FLASH_ERR;
    state = SS_FLASH;
    break;

  case SS_FLASH:  // flash the LED for end of sequence
    DEBUGS("\nSS_RECORD");
    doFlasher(flashCount, FLASH_PERIOD / flashCount, curSelection);
    state = SS_END;
    break;

  case SS_END:
    DEBUGS("\nSS_END");
  default:
    state = SS_IDLE;
  }
}

void modeRun(void)
// Code runs when the remote is in run mode (most of the time)
// See comments at the file header for how this mode works.
{
  static runState_t state = RS_INIT;  // FSM state variable
  static uint8_t  curSelection = 0;   // once we have a selection, remember it
#ifdef UMOTE_DBG
  static boolean  bInIdle = false;    // blocking sentinel to avoid reams of debug output
#endif

  switch (state)
  {
  case RS_INIT: 
    DEBUGS("\nRS_INIT");
    state = RS_SLEEP;
    break;

  case RS_SLEEP:
    DEBUGS("\nRS_SLEEP");
#ifdef USE_SLEEP
    sleepNow();
#endif
    state = RS_IDLE;
    break;

  case RS_IDLE: // Waiting for a key to be pressed
#ifdef UMOTE_DBG
    if (!bInIdle) DEBUGS("\nRS_IDLE");
    bInIdle = true;
#endif // RS_IDLE
    // If none of the switches has been pressed, just keep looping
    if ((curSelection = checkInputSignals()) == NO_INPUT)
    {
      // DEBUGS("\nNothing to process");
      break;
    }

    // One of the switches has been pressed, need to transmit something ...
    DEBUG("\nSending sw", curSelection);
    state = RS_SEND;
#ifdef UMOTE_DBG
    bInIdle = false;
#endif
    break;

  case RS_SEND: // Send the code associated with the current switch selection
    DEBUGS("\nRS_SEND");

    // read the data from eeprom
    eepromRead(curSelection, &irMesg);

    if ((irMesg.type != UNKNOWN) || (irMesg.dataLen != 0))
    {
      setFlasher(LED_ON, curSelection);  // set the LED while transmitting

      for (uint8_t i = 0; i < SEND_REPEATS; i++)
      {
        if (irMesg.type != UNKNOWN)
          irSend.send(irMesg.type, irMesg.value[0], irMesg.value[1]);
        else
          if (irMesg.dataLen != 0)  // do we actually have data to send?
            irSend.IRsendRaw::send(&irMesg.data[1], irMesg.dataLen - 1, IRSENDKHZ);  // first element is the gap before mark
        delayMicroseconds(SEND_DELAY);
      }
    }
    state = RS_WAIT1;
    break;

  case RS_WAIT1: // Set up wait for WAIT_DELAY time between processing keypresses
    S[curSelection].timeLast = millis();  // for the wait delay
    DEBUGS("\nRS_WAIT start");
    state = RS_WAIT2;
    break;

  case RS_WAIT2:  // Execute WAIT_DELAY time between processing keypresses
    if (millis() - S[curSelection].timeLast >= WAIT_DELAY)
    {
      DEBUGS("\nRS_WAIT end");
      setFlasher(LED_OFF, curSelection);  // clear the LED after transmitting
      state = RS_END;
    }
    break;

  case RS_END:
    DEBUGS("\nRS_END");
  default:
    state = RS_SLEEP;
  }
}

void establishMode(void)
// Establish the current run mode depending on what switch combination
// is active at power up. Called during setup() after the I/O has been
// initialised.
{
  // check the status
  if ((digitalRead(PROG_PIN) == SWITCH_ACTIVE) && 
      (digitalRead(CLEAR_PIN) == SWITCH_ACTIVE))
  {
    DEBUGS("\nClear mode");
    doFlasher(2, FLASH_PERIOD / 2);  // flash all the feeback leds
    runMode = MODE_CLR;
  }
  else if ((digitalRead(PROG_PIN) == SWITCH_ACTIVE) ||
    (digitalRead(CLEAR_PIN) == SWITCH_ACTIVE))
  {
    DEBUGS("\nProgram mode");
    doFlasher(1, FLASH_PERIOD);      // flash all the feeback leds
    runMode = MODE_PROG;
  }
  else
  {
    DEBUGS("\nRun mode");
    // Flash each LED in turn as a lamp test and to show startup
    for (uint8_t i = 0; i < MAX_SWITCH; i++)
      doFlasher(1, FLASH_PERIOD/MAX_SWITCH, i);
    runMode = MODE_RUN;
  }

  // Busy until the switches are unpressed. This avoids Umote executing the
  // 
  while ((digitalRead(PROG_PIN) == SWITCH_ACTIVE) ||
        (digitalRead(CLEAR_PIN) == SWITCH_ACTIVE))
    /* do nothing - just wait */;
}

void setup()
// Initialisation code
{
#ifdef UMOTE_DBG
  Serial.begin(57600);
#endif // UMOTE_DBG
  DEBUGS("\n\n[Umote Debug]");
  DEBUG("\nIR IRQ on pin ", Pin_from_Intr(IR_RECV_IRQ));

  // work out if EEPROM is large enough for holding config data
  if (sizeof(irCode_t) * ARRAY_SIZE(S) > EEPROM.length())
  {
    DEBUG("\n** WARNING: EEPROM can overflow - need ", sizeof(irCode_t) * ARRAY_SIZE(S));
    DEBUG(", have ", EEPROM.length());
  }

  // Set up switch I/O pins
  pinMode(PROG_PIN, INPUT_PULLUP);
  pinMode(CLEAR_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < MAX_SWITCH; i++)
  {
    // set up the I/O, interrupts, etc
    if (S[i].pinSignal != NO_PIN)
    {
      pinMode(S[i].pinSignal, INPUT_PULLUP);
#ifdef USE_SLEEP
      pciSetup(S[i].pinSignal); // set up interrupt
#endif
#ifdef UMOTE_DBG
      eepromRead(i, &irMesg); // just to dump the key values
#endif
    }
    if (S[i].pinFeedback != NO_PIN) 
      pinMode(S[i].pinFeedback, OUTPUT);
  }

  // establish the mode we are running
  establishMode();  

  // IR library setup
  irRecv.No_Output();
}

void loop()
// dispatcher routine for the mode being run
{
  switch (runMode)
  {
    case MODE_RUN:  modeRun();  break;
    case MODE_PROG: modeProg(); break;
    case MODE_CLR:  modeClr();  break;
  }
}
