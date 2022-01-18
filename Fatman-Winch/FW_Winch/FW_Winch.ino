// File: FW_Winch.ino
// Control of Motor for Cover Winch on Huge LaserCutter
// By janbee@hack42.nl jan@breem.nl original:july 2019
// Target: Arduino Nano

#define VERSION "2019-07-21" 

// Motor (24V windshield whiper type) is driven by 10-bit PWM from a 24 Volts power supply.
// Motor Driver is a IBT module with two drivers BTS7960 
// Motor voltage is 24V * (512 - PWM) / 512, where PWM = 0 .. 511
// Motor current is sensed with an ACS712 - 20A sensor, produces 100 mV/A, around VCC/2
// We have positive feedback of the motor current to the motor voltage.
// When the motor is off the offset of the current sensor is measured.
// Potmeter inputs are averaged by leaky bucket algorithm using a fixed point algorithm.
// This averaging is also used to produce soft start and stop.

// Arduino Pin Use:
// pin D2, PORTD2, Button_Open, active low, has internal pullup
// pin D3, PORTD3, Button_Close, active low, has internal pullup
// Endswitches open when hit. A loose wire results in no movement.
// pin D4, PORTD4, Endswitch Open, High = hit endswitch, has pullup with LED
// pin D5, PORTD5, Endswitch Close, High = hit endswitch, has pullup with LED
// pin D6, PORTD6, RXD SoftwareSerial, receives commands from SafetyBoard
// pin D7. PORTD7, TXD SoftwareSerial, sends Status to SafetyBoard
// pin D9, PORTB1, PWM output OCR1B
// pin D10, PORTB2, PWM output OCR1A
// pin D11, PORTB3, Enable Drivers, active high
// pin D12, PORTB4, Diagnose ISR, active high
// pin D13, PORTB5, Communication with SafetyBoard (SoftwareSerial) alive led on Nano
// pin A0, ADC0, potmeter Velocity
// pin A1, ADC1, potmeter Feedback
// pin A2, ADC2, motor current 
// Pin RX0, TX0: Programming and diagnostics. Normally not used
// Voltages in Volts, Currents in A, Resistances in Ohm

#define PowerSupplyVoltage 24.0
#define ADRange 1024
#define ADReferenceVoltage 5.0
#define MotorResistance 0.5
#define CurrentSensitivity 0.1 // V/A
#define CurrentLimit_mA 2000 
#define CURRENTTIMEOUT 200; // 2 seconds  holdoff before measuring the current sensor's offset

// Dataflow:
// VelocityPotmeter > ADC > adVelocityPot > averagedVelocityPot > DesiredVelocityCal > DesiredVelocity [0 ..12000]
// FeedbackPotmeter > ADC > adFeedbackPot > averagedFeedbackPot > Feedback [0 .. 1024, nominal 512]
// MotorCurrent > CurrentSensor > ADC > adMotorCurrent > averagedMotorcurrent -Offset > MotorCurrentCal > MotorCurrent
// MotorVoltage = DesiredVelocity + (FeedBack * MotorCurrent / 512)
// PWM = DesiredMotorVoltage / PowerSupplyVoltage * 1024

#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(7, 6); // RX, TX

int adVelocityPot, adFeedbackPot, adMotorCurrent;
int averagedVelocityPot, averagedFeedbackPot, averagedCurrentOffset;
int VelocityPot, FeedbackPot, CurrentOffset; // range 0..1023
const float CurrentCalibration = - ADReferenceVoltage / ADRange / CurrentSensitivity; 
float MotorCurrent, Feedback, DesiredVelocity, MotorVoltage; 
int MotorPWM, MotorCurrent_mA, averagedMotorCurrent_mA;
bool OverCurrent = false;
enum ButtonStates {PON, IsFullyOpen, IsClosed, IsInMiddle, IsOpening, 
                  IsClosing, IsOverCurrent};
ButtonStates ButtonState;                  
byte DebounceButtOpen, DebounceButtClose;
bool ButtOpenPressed, PrevButtOpenPressed, ButtClosePressed, PrevButtClosePressed;
byte Timer2State, HoldOffTime;
bool HoldOff, SeenOverCurrent;
char StatusChar;
int  rpCurrent;

// Outputs:
#define DiagH PORTB |= 0x10;  // Diagnostic on pin D12
#define DiagL PORTB &= ~0x10;
#define DiagT PINB |= 0x10;
#define CommunicationLedToggle PINB |= 0x20;
#define MotorEnable PORTB |= 0x80;
#define MotorDisable PORTB &= ~0x80;

// Inputs:
#define hwButtOpenPressed ((PIND & 0x04) == 0)
#define hwButtClosePressed ((PIND & 0x08) == 0)
// Endswitches connect to GND and open when hit. A broken wire results in hit = no movement in that direction.
#define EndSwitchOpenHit ((PIND & 0x10) == 0x10)
#define EndSwitchCloseHit ((PIND & 0x20) == 0x20)

//********************************************************************************************************
void EvaluateButtons (void) // called at 100 Hz rate
{
  // Debounce buttons and detect change to pressed only
  if(DebounceButtOpen < 10)  if(hwButtOpenPressed) DebounceButtOpen++;
  if(DebounceButtOpen > 0)   if(!hwButtOpenPressed)  DebounceButtOpen--; 
  if(DebounceButtOpen == 10) if(!PrevButtOpenPressed) {ButtOpenPressed = true; PrevButtOpenPressed = true;}
  if(DebounceButtOpen == 0)  PrevButtOpenPressed = false;

  if(DebounceButtClose < 10)  if(hwButtClosePressed) DebounceButtClose++;
  if(DebounceButtClose > 0)   if(!hwButtClosePressed)  DebounceButtClose--; 
  if(DebounceButtClose == 10) if(!PrevButtClosePressed) {ButtClosePressed = true; PrevButtClosePressed = true;}
  if(DebounceButtClose == 0)  PrevButtClosePressed = false;
  
  switch (ButtonState)
  { 
    case PON:
      if(EndSwitchOpenHit)
      {
        Serial.println("Is FullyOpen"); 
        StatusChar = 'O';
        ButtonState = IsFullyOpen;
        break;
      }
      if(EndSwitchCloseHit)
      {
        Serial.println("Is Closed");  
        StatusChar = 'C';       
        ButtonState = IsClosed;
        break;
      }
      Serial.println("Is in the Middle"); 
      StatusChar = 'M';
      ButtonState = IsInMiddle;
      break;
      
    case IsFullyOpen:
      if(ButtClosePressed)
      {
        ButtClosePressed = false;
        StatusChar = 'c';
        Serial.println("Was Fully Open, Now Closing"); 
        ButtonState = IsClosing;
      }
      if(!EndSwitchOpenHit)
      {
        Serial.println("Was Fully Open, Lost Endswitch, Now in Middle");
        StatusChar = 'M';
        ButtonState = IsInMiddle;
      }
      break;

    case IsClosed:
      if(ButtOpenPressed)
      {
        ButtOpenPressed = false;
        StatusChar = 'o';
        Serial.println("Was Closed, Now Opening");
        ButtonState = IsOpening;
      }
      if(!EndSwitchCloseHit)
      {
        Serial.println("Was Closed, Lost Endswitch, Now in Middle");
        StatusChar = 'M';
        ButtonState = IsInMiddle;
      }
      break;

    case IsOpening:
      if(EndSwitchOpenHit)
      {
        Serial.println("Was Opening, Hit Endswitch, Now Fully Open"); 
        StatusChar = 'O';
        HoldOffTime = CURRENTTIMEOUT
        ButtonState = IsFullyOpen;
      }
      if(ButtClosePressed)
      {
        ButtClosePressed = false;
        Serial.println("Was Opening, ButtClosePressed, Now Stopped in Middle");
        StatusChar = 'M';
        HoldOffTime = CURRENTTIMEOUT
        ButtonState = IsInMiddle;
      }
      break;
    
    case IsClosing:
      if(EndSwitchCloseHit)
      {
        Serial.println("Was Closing, Hit Endswitch, Now Closed");
        StatusChar = 'C';
        HoldOffTime = CURRENTTIMEOUT
        ButtonState = IsClosed;
      }
      if(ButtOpenPressed)
      {
        ButtOpenPressed = false;
        Serial.println("Was Closing, ButtOpenPressed, Now Stopped in Middle"); 
        StatusChar = 'M';
        HoldOffTime = CURRENTTIMEOUT
        ButtonState = IsInMiddle;
      } 
      break;

    case IsInMiddle:
      if(ButtOpenPressed)
      {
        ButtOpenPressed = false;
        Serial.println("Was Stopped in Middle, ButtOpenPressed, Now Opening"); 
        StatusChar = 'o'; 
        ButtonState = IsOpening; 
      }
      if(EndSwitchOpenHit)
      {
        Serial.println("Was in Middle, Hit Endswitch, Now Fully Open"); 
        StatusChar = 'O';
        ButtonState = IsFullyOpen;
      }
      if(ButtClosePressed)
      {
        ButtClosePressed = false;
        Serial.println("Was Stopped in Middle, ButtClosePressed, Now Closing");
        StatusChar = 'c';
        ButtonState = IsClosing;     
      }
      if(EndSwitchCloseHit)
      {
        Serial.println("Was Closing, Hit Endswitch, Now Closed");
        StatusChar = 'C';
        ButtonState = IsClosed;
      }
      break;
      
   case IsOverCurrent:
      StatusChar = 'E';
      // we leave this state only by RESET
      break;  
  } // switch - case

  if(SeenOverCurrent && (ButtonState != IsOverCurrent))
  {
    StatusChar = 'E';
    MotorDisable;
    Serial.println("Overcurrent. Needs Reset");
    ButtonState = IsOverCurrent;
  }
 
  // After we stopped we need some time to let the motorcurrent go to zero
  // before we start measuring the current-sensor's offset
  if(HoldOffTime > 0) 
  {
    HoldOff = true;
    HoldOffTime--;
  }
  else HoldOff = false;
}

//********************************************************************************************************
int Limit (int Min, int Value, int Max)
{
  if(Value > Max) return(Max);
  if(Value < Min) return(Min);
  return(Value);
}

//********************************************************************************************************
void SendData(void)
{
  CommunicationLedToggle;
  Serial.print("V: ");
  Serial.print(VelocityPot);
  Serial.print(" F: ");
  Serial.print(FeedbackPot); 
  Serial.print(" I: ");
  Serial.print(rpCurrent);
  Serial.print(" MV: ");
  Serial.print(MotorVoltage); 
  Serial.print(" PWM: ");
  Serial.print(MotorPWM); 
  Serial.print(" MI: ");
  Serial.print(MotorCurrent); 
  Serial.print(" Offs: ");
  Serial.print(CurrentOffset); 
  if(HoldOff) Serial.print(" H "); else Serial.print( " ok ");
  Serial.println(StatusChar);
}

//********************************************************************************************************
ISR(TIMER2_COMPA_vect)  // 400 Hz, so each channel 100 x per second
{
  static byte OutputCounter;
  DiagH // Pin 15, D12 shows the Interrupt-In-Service time and frequency 
  
  if(++OutputCounter > 150)
  {
    OutputCounter = 0;
    SendData(); // 2 x per second
  }

  if (SeenOverCurrent)
  {
    PORTB &= ~0x08;  // Disable Motor Driver 
    return;
  }
  // In cases 0, 1, and 2 we do A/D conversions, averaging and calibration 
	// note: ADC holds the value from the conversion which was started in the previous timeslice
  // In case 3 we update the motor PWM
  
  switch(Timer2State) 
  {
  case 0:  // update Velocity potmeter and Buttons
    EvaluateButtons();
    ADMUX = 0x41; // next is feedback potmeter 
	  adVelocityPot = ADC; // from AD channel 2, VelocityPotmeter
    if(ButtonState == IsClosing) adVelocityPot = -adVelocityPot;
    if((ButtonState != IsClosing) && (ButtonState != IsOpening)) adVelocityPot = 0;
    // averaged values are shifted 4 bit left.
    // averageing serves soft start and stop
    averagedVelocityPot = averagedVelocityPot - (averagedVelocityPot >> 4) + adVelocityPot;
    VelocityPot = averagedVelocityPot >> 4; //range 0 .. 1023
    DesiredVelocity = float(VelocityPot) * PowerSupplyVoltage / ADRange;
    Timer2State = 1;
    break;
		
  case 1: // update Feedback potmeter
    ADMUX = 0x42; // next is motor current 
    adFeedbackPot = ADC;// from AD channel 0, FeedBackPotmeter
    averagedFeedbackPot = averagedFeedbackPot - (averagedFeedbackPot >> 6) + adFeedbackPot;
    FeedbackPot = averagedFeedbackPot >> 6;
    Feedback = float(FeedbackPot) / ADRange * 2; // range = 0..2
    Timer2State = 2; 
    break;
 
  case 2: // update measured motor current
    ADMUX = 0x40; // next is velocity potmeter 
    adMotorCurrent = ADC; // from AD channel 1, Motorcurrent
    rpCurrent = adMotorCurrent;
    if ((ButtonState != IsOpening) && (ButtonState != IsClosing) && !HoldOff) // update current offset
    {
      averagedCurrentOffset = averagedCurrentOffset - (averagedCurrentOffset >> 6) + adMotorCurrent;
      CurrentOffset = (averagedCurrentOffset >> 6);  
    }
    else
    {
      MotorCurrent = (adMotorCurrent - CurrentOffset) * CurrentCalibration;
      MotorCurrent_mA = abs(round(MotorCurrent * 1000));
      // we average the motorcurrent_mA so we do not get overcurrent on short peaks
      averagedMotorCurrent_mA += (-(averagedMotorCurrent_mA >> 6) + MotorCurrent_mA);
      if((averagedMotorCurrent_mA >> 6) > CurrentLimit_mA) {SeenOverCurrent = true; exit;}
    }
    Timer2State = 3; 
    break;
    
  case 3: // update Motor control
    MotorVoltage = DesiredVelocity + MotorCurrent * MotorResistance * Feedback;  // positive feedback
    MotorPWM = round(MotorVoltage * 1024 / PowerSupplyVoltage); 
    MotorPWM = Limit (-511, MotorPWM, 511); // range -511 .. +511
    OCR1A = 512 + MotorPWM;
    OCR1B = 512 - MotorPWM; 
    Timer2State = 0; 
    break;
  }
  ADCSRA |= 0x40; // start AD conversion. Result is found on the next interrupt 
  DiagL // Pin D12 shows the Interrupt-In-Service time and frequency
}

//********************************************************************************************************
void setup(void) 
{
  Serial.begin(19200);
  Serial.print ("Motor Control for Cover of LaserCutter v: ");
  Serial.println(VERSION);
  
  SoftSerial.begin(19200);
   
  DDRB |= 0x10;   // diagnostic output PORT B4, PIN 15
  PORTD |= 0x0C;  // internal pullups for buttons on pin D2 and D3
  
  // configure 8-bit Timer2 for 400 Hz Interrupt ISR(TIMER2_COMPA_vect)
  TCCR2A = 0x02;  // Waveform mode 2: CTC
  TCCR2B = 0x07;  // internal clock, prescaler 1024, WaveformMode 2: CTC
  OCR2A = 38;     // 400 Hz
  TIMSK2 = 0x02;  // interrupt on OCRA match   
 
  // configure 16 bit Timer1 for 10 bit Fast PWM (~16 kHz) on channel A and B
  TCCR1A = 0xA3;  // Waveform mode 7, Clear on Compare Match
  TCCR1B = 0x09;  // internal clock, prescaler 1, WaveformMode 7
  OCR1A = 512;    // Motor starts on 0% = no movement  
  OCR1B = 512;    // Motor starts on 0% = no movement   
  DDRB |= 0x0E;   // activate OCR1A, OCRBB PWM and Enable port B1, B2 pin D9, D10, 11 
  PORTB |= 0x08;  // Enable Motor Driver
  
  // Prepare A/D converter for 10 bit conversions
  ADMUX = 0x40;   // Ref = AVCC, 10 bit data right justified
  ADCSRA = 0x86;  // Enable A/D converter, no interrupts, 
                  // clock / 64 = 0.25 MHz for ca. 28 usec conversion time
  ADCSRB = 0x08;  // channels 0 .. 2 Single ended mode
  DIDR0 = 0x7;    // Disable digital I/O on ADC0 .. ADC2
  
  CurrentOffset = 511; // initial value. Will be updated when no movement.
  ButtonState = PON;
}

//********************************************************************************************************
void loop(void)
{
  static char PrevInChar;
  if(SoftSerial.available()) 
  {
    char InChar = SoftSerial.read();
    if(InChar == 'O') ButtOpenPressed = true;
    if(InChar == 'C') ButtClosePressed = true;
    if(InChar != PrevInChar)
    {
      Serial.print("Got command ");
      Serial.println(InChar);
      PrevInChar = InChar;
    }
    SoftSerial.write(StatusChar);
  }
  if(Serial.available())
  {
    char InChar = Serial.read();
    if(InChar == 'O') SeenOverCurrent = true; // for testuing only
  }
}
