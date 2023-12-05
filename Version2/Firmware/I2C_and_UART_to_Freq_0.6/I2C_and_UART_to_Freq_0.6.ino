

/* Software for Zachtek "RF Syntesizer 2" 
   
   Arduino software that takes incoming data on serial port or I2C and sets the output frequency on a Synthesizer
   Uses Address 11 on I2C bus.
   Portions of the code is based on work done for ADF4351 by Alain Fort F1CJN 
   The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
   Uses ARduino-SerialCommand library https://github.com/kroimon/Arduino-SerialCommand
   
   ***********************************************************************
   *To compile tis first set the board to "Arduino Pro or Pro Mini".
   *Next set the Processor to "ATMega328P (3.3V, 8MHz)"
   *Add the SerialCommand library from this URL https://github.com/kroimon/Arduino-SerialCommand
   ***********************************************************************
   
  
   
   In and Outputs
     2  In   --  Ext ref detect.          Not used in this firmware
    10  In   --  Latch enable on ADF4351, Used by the SPI routine 
    11  Out  --  SPI Data                 Used by the SPI routine
    13  Out  --  SPI Clock                Used by the SPI routine
    A0  In   --  Lock detect from ADF4351 (Mux Out, can be reprogramed for other use)  
    A2  Out  --  Yellow Status LED that indicates Start up. 
    RXI In   --  Used by the Serial routine
    TXO Out  --  Used by the Serial routine 
    

*/




//************************************ Declarations  ****************************************
#include "Wire.h"
#include <SPI.h>
#include <SerialCommand.h>
SerialCommand SCmd;           // The SerialCommand object

#define ADF4351_LE 10    //Latch enable on pin 10
#define ADF4351_LD A0    //Lock Detect (MUXOUT) on pin  A0
#define  StatusLED A2    //LED to indicator for Boot and Command received on pin A2


//uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC83FC, 0x580005} ; // Init registers with RF A and B On at full level
uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x14004E42, 0x4B3, 0xBC803C, 0x580005} ; // Init registers with RF A On at full level and RF B Off


unsigned int i = 0;
double RFout, REFin, INT, OutputChannelSpacing, FRACF;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider;
unsigned int long reg0, reg1;

uint64_t MaxFreq=4400000000;
uint64_t MinFreq=138000000;
uint64_t ADF4351SetFreq=2350000000; // Set startfrequency to 2.35GHz

uint64_t Sweepstart=250000000;
uint64_t Sweepstop =4400000000;
uint64_t Sweepstep =1000000;
boolean  sweeping=false;

boolean CommandQuery=false;
boolean WaitLock=false;
int LockTimeout=300; 
double PFDRFout=10; //The Frequency of the reference oscillator, If you use the external input with anything else than 10MHz then change this value to the ref. Freq in MHz

const char softwareversion[] = "0.6" ; //Version of this program, sent to serialport at startup




//************************************ Setup ****************************************
void setup() {
  Serial.begin (9600);
  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand("SETFREQ",SCMDSetFreq);           // Sets Synthesizer output frequency
  SCmd.addCommand("INC",SCMDInc);                   // Increments frequency 
  SCmd.addCommand("DEC",SCMDDec);                   // Decrements frequency
  SCmd.addCommand("GETFREQ",SCMDGetFreq);           // Writes current frequency

  SCmd.addCommand("SWLOW",SCMDSweepLo);             // Sets Sweep start frequency
  SCmd.addCommand("SWHIGH",SCMDSweepHigh);          // Sets Sweep stop frequency
  SCmd.addCommand("SWSTEP",SCMDSweepStep);          // Sets Sweep steps
  SCmd.addCommand("SWSTART",SCMDStartSweep);        // Start sweeping
  SCmd.addCommand("SWSTOP",SCMDStopSweep);          // Stop sweeping
  
  SCmd.addCommand("GETLOCK",SCMDGetLock);           // Gives PLL lock status
  SCmd.addCommand("RFON",SCMDRFAOn);                // Turns on or off the output buffers on the synthesizer
  SCmd.addCommand("RFOFF",SCMDRFAOn);               // Mutes the RF A outputs
  //SCmd.addCommand("VOLTON",SCDMPowerOn);            // Turns on voltage regulator to syntehsizer and program it to the last freq
  //SCmd.addCommand("VOLTOFF",SCDMPowerOff);          // Turns off voltage regulator to synthesizer.

  SCmd.addCommand("HELP",SCMDHelp);                 // Displays help page 
  
  SCmd.setDefaultHandler(SCMDunrecognized);         // Handler for command that isn't matched 
  
  Wire.begin(11);                  // join i2c bus with address #11 as a I2C Slave unit
  Wire.onReceive(I2CreceiveEvent); // register event so we can receive data from I2C bus
  //delay (300);
  
  Serial.print(F("Zachtek RF Syntesizer 2 Software version: "));
  Serial.println(softwareversion);
  Serial.println(F("Type HELP in upper-case and press enter for command information"));
  Serial.println(F("(NL+CR must be sent for the commands to be recognized)"));
  Serial.println("");
  
  //pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
  //Serial.println ("Turning on Voltage Regulators for ADF4351 IC");
  //digitalWrite(LDO_Enable, HIGH); //Turn on Power supply for the ADF4351 IC

  pinMode(StatusLED, OUTPUT); // Set Command LED pin as output.
  digitalWrite(StatusLED, LOW); //Turn off Status LED
  
  pinMode(ADF4351_LD, INPUT);  // Set Lock Detect input pin
  pinMode(ADF4351_LE, OUTPUT); // Set Latch Enable output pin
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  delay (100);
   
 
 
  SetADFFreq ();
  Serial.print ("Frequency set to ");
  Serial.print (uint64ToStr(ADF4351SetFreq,false));
  Serial.println ("Hz");

    //Blink  to indicate Reboot
  LEDBlink(16);
  
} // end setup


//************************************* Main Loop***********************************
void loop()
{


    SCmd.readSerial();     //process serial commands

    //Pulse LED twice if we got a command on the UART or I2C
    if (CommandQuery) {
      LEDBlink(2);
      CommandQuery=false; 
    }

  //Sweep if commanded to do so
  if (sweeping) {
    if (!WaitLock) //If we are not waiting for the PLL to Lock
      {
        ADF4351SetFreq=ADF4351SetFreq+Sweepstep;  //Calc next Swep frequency
        if (ADF4351SetFreq>Sweepstop) //If reached end, start over the sweep
        { 
          ADF4351SetFreq=Sweepstart;   //Set new freq
        }
        SetADFFreq ();
        WaitLock=true;               //We will now wait for the PLL to lock 
        LockTimeout=1000;
       }
       else  //We are waiting for the PLL to Lock
       {   
         LockTimeout=LockTimeout-1;
        // Serial.println ("Waiting for PLL lock");
         if (digitalRead (ADF4351_LD)==HIGH) {
           WaitLock=false; //Stop waiting when MUX output goes HIGH = PLL Lock
          // Serial.println ("PLL is locked");
         }
         if (LockTimeout <= 0) {
            WaitLock=false;  // Alterantive stop waiting for lock after some time
          //  Serial.println ("PLL lock Timeout");
         }
       }
       delay (10);
   }   
}



//************************************ Functions and Procedures ****************************************
void WriteRegister32(const uint32_t value)   //Write a ADF4351 Register
{
  digitalWrite(ADF4351_LE, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));// Normal SPI
  //SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0)); //Slow SPI for debugging with Logic Analyzer
  for (int i = 3; i >= 0; i--) 
  {         // boucle sur 4 x 8bits
    SPI.transfer((value >> 8 * i) & 0xFF); // 
    //delayMicroseconds(120); //Delay for debugging
  }
  SPI.endTransaction();
  digitalWrite(ADF4351_LE, HIGH);
  //delayMicroseconds(100);  //Delay for debugging
 
}

void SetADF4351()  
{ for (int i = 5; i >= 0; i--) 
    WriteRegister32(registers[i]);
}

void SetADFFreq ()
{
  RFout=ADF4351SetFreq/1000000;

    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    
    RFout=ADF4351SetFreq;
    INTA = (RFout * OutputDivider) / (PFDRFout*1000000);
    MOD = PFDRFout;
    FRACF = (((RFout * OutputDivider) / (PFDRFout*1000000)) - INTA) * MOD;
    FRAC = round(FRACF); // 
    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;
    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; 
    bitSet (registers[1], 27); 
    bitSet (registers[2], 28); 
    bitSet (registers[2], 27); 
    bitClear (registers[2], 26); 
  
    SetADF4351();  
   // RFintold=RFint;modif=0;
 // }
}


//Brief flash on the Status LED 'Blinks'" number of time
void LEDBlink(int Blinks)
{
  for (int i = 0; i < Blinks; i++)
  {
    digitalWrite(StatusLED, HIGH);
    delay (50);
    digitalWrite(StatusLED, LOW);
    delay (50);
  }
}

void SCMDSetFreq() {
char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  { 
    ADF4351SetFreq = StrTouint64_t(arg);
   if (ADF4351SetFreq > MaxFreq)//Bounds check
  {
    ADF4351SetFreq = MaxFreq; 
  }
   if (ADF4351SetFreq < MinFreq)//Bounds check
  {
    ADF4351SetFreq = MinFreq; 
  }
  SetADFFreq ();
  Serial.print("F=");
  Serial.println(uint64ToStr(ADF4351SetFreq,false));
  CommandQuery=true;
  } 
  else {
    Serial.println(F("Missing frequency value")); 
  }
}

void SCMDInc() {
char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  { 
     ADF4351SetFreq = ADF4351SetFreq + StrTouint64_t(arg);
     if (ADF4351SetFreq > MaxFreq)//Bounds check
     {
       ADF4351SetFreq = MaxFreq; 
     }
  
    SetADFFreq ();
    Serial.print("+");
    Serial.println(arg);
    CommandQuery=true;
  } 
  else 
    {
    Serial.println(F("Missing increment value")); 
  }
}

void SCMDDec() {
  uint64_t decNumber;
  char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
    { 
     decNumber = StrTouint64_t(arg);
     if (decNumber > ADF4351SetFreq) //If decrementing with this value will get a negative number then set to 0 to avoid wrapping around bounds
       {
       ADF4351SetFreq = 0; 
       }
     else
      {
      ADF4351SetFreq = ADF4351SetFreq - decNumber;
     }
     //Check that we are not below min freq possible
     if (ADF4351SetFreq < MinFreq)//Bounds check
       {
       ADF4351SetFreq = MinFreq; 
     }
     SetADFFreq ();
     Serial.print("-");
     Serial.println(arg);
    }
   else
   {
    Serial.println(F("Missing decrement value"));  
   }
 CommandQuery=true;
}



void SCMDGetFreq () {
  Serial.print(F("F=")); 
  Serial.println(uint64ToStr(ADF4351SetFreq, false));
  CommandQuery=true;
}


void SCMDGetLock () {
  int lockstate;
  lockstate=digitalRead(ADF4351_LD);
   if (lockstate==HIGH) {
     Serial.println("Locked");
   }
   else
   {
     Serial.println("Unlocked"); 
   }
   CommandQuery=true;
}


void SCMDRFAOn() 
{
 Serial.println("Not Implemented");
}

void SCMDRFAOff() 
{
 Serial.println("Not Implemented");
}


void SCDMPowerOn() {
  //digitalWrite(LDO_Enable,HIGH); 
  delay(1);
  SetADFFreq ();
  Serial.println(F("Voltage Regulators turned on"));
  CommandQuery=true; 
}


void SCDMPowerOff() {
  //digitalWrite(LDO_Enable,LOW); 
  Serial.println(F("Voltage Regulators turned off"));
  CommandQuery=true; 
}


void SCMDSweepLo () {
char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  { 
    Sweepstart = StrTouint64_t(arg);
   if (Sweepstart > MaxFreq)//Bounds check
  {
    Sweepstart = MaxFreq; 
  }
   if (Sweepstart < MinFreq)//Bounds check
  {
    Sweepstart = MinFreq; 
    }
    Serial.print(F("Sweep Low Freq set to "));
    Serial.println(uint64ToStr(Sweepstart,false));
  } 
  else 
    {
    Serial.println(F("Missing frequency value for Sweep")); 
  }
  CommandQuery=true;
}


void SCMDSweepHigh () {
char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
    { 
    Sweepstop = StrTouint64_t(arg);
    if (Sweepstop > MaxFreq)//Bounds check
      {
      Sweepstart = MaxFreq; 
    }
    if (Sweepstop < MinFreq)//Bounds check
      {
      Sweepstop = MinFreq; 
      }
      Serial.print(F("Sweep High Freq set to "));
      Serial.println(uint64ToStr(Sweepstop,false));
    } 
  else 
    {
    Serial.println(F("Missing frequency value for Sweep")); 
  }
  CommandQuery=true;
}


void SCMDSweepStep () {
char *arg;  
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  { 
    Sweepstep = StrTouint64_t(arg);
    Serial.print(F("Sweep Stepsize set to "));
    Serial.println(uint64ToStr(Sweepstep,false));
  } 
  else {
    Serial.println(F("Missing frequency value for Sweep")); 
  }
   CommandQuery=true;
}

void SCMDStartSweep () {
    ADF4351SetFreq = Sweepstart; 
    sweeping=true;
    Serial.print (F("Sweep started from "));
    Serial.print (uint64ToStr(Sweepstart,false));
    Serial.print (F(" to "));
    Serial.print (uint64ToStr(Sweepstop,false));
    Serial.print (F(" stepsize  "));
    Serial.println (uint64ToStr(Sweepstep,false));
    Serial.println (F("Enter SWSTOP to stop the sweep"));
   CommandQuery=true;
}
  
void SCMDStopSweep () {
    sweeping=false;
    Serial.println (F("Sweep has been stoped"));
   CommandQuery=true;
} 

   

void SCMDHelp () {

  Serial.println(F("Type one of the following commands to control the Synthesizer: "));
  Serial.println(F(" : SETFREQ ...  , sets a new frequency in the Synthesizer, value in Hz"));
  Serial.println(F(" : INC ...  , Increment the frequency.  e.g. 'Inc 15000' will increment the existing frequency with 15kHz"));
  Serial.println(F(" : DEC ...  , Decrements the frequency"));
  Serial.println(F(" : GETFREQ  , Shows the Frequency that is currently set in the Syntesizer"));
  Serial.println("");
  Serial.println(F(" : SWLOW    , Sets the lower frequency for frequency sweep"));
  Serial.println(F(" : SWHIGH   , Sets the higher frequency for frequency sweep"));
  Serial.println(F(" : SWSTEP   , Sets stepsize for the sweep, e.g SWSTEP 100000 sets a sweep step of 100kHz"));
  Serial.println(F(" : SWSTART  , Starts sweeping"));
  Serial.println(F(" : SWSTOP   , Stops the sweep"));
  Serial.println("");
  Serial.println(F(" : GETLOCK  , shows if the PLL is locked or not (LOCK or UNLOCK)"));
  Serial.println(F(" : RFON     , Turns on RF Output A+ and A- on the synth."));
  Serial.println(F(" : RFOFF    , Mutes the RF A+ and A- outputs but keep the PLL locked internally."));
  // Serial.println(F(" : VOLTON   , Turns on voltage regulator to syntehsizer and program it to the last freq."));
  // Serial.println(F(" : VOLTOFF  , Turns off the power supply to the synth."));
  Serial.println(F(" : HELP     ,prints this information"));
  Serial.println("");
  Serial.println(F(" Commands are case sensitive, only use UPPER CASE"));
  Serial.println("");
  CommandQuery=true; 
}

/*  SCmd.addCommand("SETFREQ",SCMDSetFreq);           // Sets Synthesizer output frequency
  SCmd.addCommand("INC",SCMDInc);                   // Increments frequency 
  SCmd.addCommand("DEC",SCMDDec);                   // Decrements frequency
  SCmd.addCommand("GETFREQ",SCMDGetFreq);           // Writes current frequency

  SCmd.addCommand("SWLOW",SCMDSweepLo);             // Sets Sweep start frequency
  SCmd.addCommand("SWHIGH",SCMDSweepHigh);          // Sets Sweep stop frequency
  SCmd.addCommand("SWSTEP",SCMDSweepStep);          // Sets Sweep steps
  SCmd.addCommand("SWON",SCMDStartSweep);           // Start sweeping
  SCmd.addCommand("SWOFF",SCMDStopSweep);           // Stop sweeping
  
  SCmd.addCommand("GETLOCK",SCMDGetLock);           // Gives PLL lock status
  SCmd.addCommand("RFON",SCMDRFAOn);                // Turns on or off the output buffers on the synthesizer
  SCmd.addCommand("RFOFF",SCMDRFAOn);               // Mutes the RF A outputs
  SCmd.addCommand("VOLTON",SCDMPowerOn);            // Turns on voltage regulator to syntehsizer and program it to the last freq
  SCmd.addCommand("VOLTOFF",SCDMPowerOff);          // Turns off voltage regulator to synthesizer.
  */

// SerialCommand handler for unrecognized commands
void SCMDunrecognized(const char *command) {
   Serial.println(F("Not a valid Command")); 
}

uint64_t  StrTouint64_t (String InString)
{
  uint64_t y = 0;

  for (int i = 0; i < InString.length(); i++) {
    char c = InString.charAt(i);
    if (c < '0' || c > '9') break;
    y *= 10;
    y += (c - '0');
  }
  return y;
}

String  uint64ToStr (uint64_t p_InNumber, boolean p_LeadingZeros)
{
  char l_HighBuffer[7]; //6 digits + null terminator char
  char l_LowBuffer[7]; //6 digits + null terminator char
  char l_ResultBuffer [13]; //12 digits + null terminator char
  String l_ResultString = "";
  uint8_t l_Digit;

  sprintf(l_HighBuffer, "%06lu", p_InNumber / 1000000L); //Convert high part of 64bit unsigned integer to char array
  sprintf(l_LowBuffer, "%06lu", p_InNumber % 1000000L); //Convert low part of 64bit unsigned integer to char array
  l_ResultString = l_HighBuffer;
  l_ResultString = l_ResultString + l_LowBuffer; //Copy the 2 part result to a string

  if (!p_LeadingZeros) //If leading zeros should be romeved
  {
    l_ResultString.toCharArray(l_ResultBuffer, 13);
    for (l_Digit = 0; l_Digit < 12; l_Digit++ )
    {
      if (l_ResultBuffer[l_Digit] == '0')
      {
        l_ResultBuffer[l_Digit] = ' '; // replace zero with a space character
      }
      else
      {
        break; //We have found all the leading Zeros, exit loop
      }
    }
    l_ResultString = l_ResultBuffer;
    l_ResultString.trim();//Remove all leading spaces
  }
  return l_ResultString;
}


// function that executes whenever data is received from I2C master
// this function is registered as an event, see setup()
void I2CreceiveEvent(int howMany) {
byte count;
   for (count=0; count < (howMany); count++){ 
  //  LEDdigit[count] = Wire.read();      
  }
  CommandQuery=true;
}
