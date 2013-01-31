#include <ctype.h>
#include <HardwareSerial.h>
#include <avr/pgmspace.h>
#include "WProgram.h"
#include "vectors.h"
#include "configuration.h"
#include "intercom.h"
#include "pins.h"
#include "extruder.h"
#include "cartesian_dda.h"

#if MOTHERBOARD == 2
  #ifdef GEN6
    #include "Temperature.h"
    int extrTargetCelsius;
    volatile int currentTemperature;
  #endif
#endif

/**

RepRap GCode interpreter.

IMPORTANT

Before changing this interpreter, read this page:

http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

*/

// Yep, this is actually -*- c++ -*-

// Sanguino G-code Interpreter
// Arduino v1.0 by Mike Ellery - initial software (mellery@gmail.com)
// v1.1 by Zach Hoeken - cleaned up and did lots of tweaks (hoeken@gmail.com)
// v1.2 by Chris Meighan - cleanup / G2&G3 support (cmeighan@gmail.com)
// v1.3 by Zach Hoeken - added thermocouple support and multi-sample temp readings. (hoeken@gmail.com)
// Sanguino v1.4 by Adrian Bowyer - added the Sanguino; extensive mods... (a.bowyer@bath.ac.uk)
// Sanguino v1.5 by Adrian Bowyer - implemented 4D Bressenham XYZ+ stepper control... (a.bowyer@bath.ac.uk)
// Sanguino v1.6 by Adrian Bowyer - implemented RS485 extruders
// Arduino Mega v1.7 by Adrian Bowyer




// Maintain a list of extruders...

extruder* ex[EXTRUDER_COUNT];
byte extruder_in_use = 0;

// Text placed in this (terminated with 0) will be transmitted back to the host
// along with the next G Code acknowledgement.
char debugstring[100];


// Old Mothers...

#if MOTHERBOARD < 2

// TODO: For some reason, if you declare the following two in the order ex0 ex1 then
// ex0 won't drive its stepper.  They seem fine this way round though.  But that's got
// to be a bug.

#if EXTRUDER_COUNT == 2            
static extruder ex1(EXTRUDER_1_MOTOR_DIR_PIN, EXTRUDER_1_MOTOR_SPEED_PIN , EXTRUDER_1_HEATER_PIN,
              EXTRUDER_1_FAN_PIN,  EXTRUDER_1_TEMPERATURE_PIN, EXTRUDER_1_VALVE_DIR_PIN,
              EXTRUDER_1_VALVE_ENABLE_PIN, EXTRUDER_1_STEP_ENABLE_PIN);            
#endif

static extruder ex0(EXTRUDER_0_MOTOR_DIR_PIN, EXTRUDER_0_MOTOR_SPEED_PIN , EXTRUDER_0_HEATER_PIN,
            EXTRUDER_0_FAN_PIN,  EXTRUDER_0_TEMPERATURE_PIN, EXTRUDER_0_VALVE_DIR_PIN,
            EXTRUDER_0_VALVE_ENABLE_PIN, EXTRUDER_0_STEP_ENABLE_PIN);         
#endif

// Standard Mendel

#if MOTHERBOARD == 2

#if EXTRUDER_COUNT == 2    
static extruder ex1(E1_NAME, E1_STEPS_PER_MM);            
#endif

static extruder ex0(E0_NAME, E0_STEPS_PER_MM);

intercom talker;

#endif

// Arduino Mega

#if MOTHERBOARD == 3

#if EXTRUDER_COUNT == 2            
static extruder ex1(EXTRUDER_1_STEP_PIN, EXTRUDER_1_DIR_PIN, EXTRUDER_1_ENABLE_PIN, EXTRUDER_1_HEATER_PIN, EXTRUDER_1_TEMPERATURE_PIN);            
#endif

static extruder ex0(EXTRUDER_0_STEP_PIN, EXTRUDER_0_DIR_PIN, EXTRUDER_0_ENABLE_PIN, EXTRUDER_0_HEATER_PIN, EXTRUDER_0_TEMPERATURE_PIN); 

#endif

// Each entry in the buffer is an instance of cartesian_dda.

cartesian_dda* cdda[BUFFER_SIZE];

static cartesian_dda cdda0;
static cartesian_dda cdda1;
static cartesian_dda cdda2;
static cartesian_dda cdda3;

volatile byte head;
volatile byte tail;
bool led;

word interruptBlink;

// Where the machine is from the point of view of the command stream

FloatPoint where_i_am;

// Our interrupt function

ISR(TIMER1_COMPA_vect)
{
  disableTimerInterrupt();
  
  interruptBlink++;
  if(interruptBlink == 0x280)
  {
     blink();
     interruptBlink = 0; 
  }

      
  if(cdda[tail]->active())
      cdda[tail]->dda_step();
  else
      dQMove();
 
  enableTimerInterrupt();
}

#if MOTHERBOARD == 2
  #ifdef GEN6
    // Timer 2 interrupt used for timebase of realtime temperature controller in GEN6
    ISR(TIMER2_OVF_vect)
    {
      static int tempControllerState = TEMPREG_INIT;
      static int adcResult = 0;
      byte i;

      const int dt = 100;    //ms
      const float pGain = 20.0;
      const float iGain = 0.1;
//      const float dGain = 60.0;
      
//      static int temp_dState = 0;
      static long temp_iState = 0;
      int output;
      int error;
      float pTerm, iTerm;
//    float dTerml;
      
      pinMode(16, OUTPUT);
      digitalWrite(16, !digitalRead(16));
      
      switch(tempControllerState) {
        case TEMPREG_INIT:
          //Timer is already running by executing setupTempReg(). Now first setup AD convertor:
          ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
          DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
          ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
          tempControllerState = TEMPREG_ACQ1; //Next state
          break;
        case TEMPREG_ACQ1:
          if (ADCSRA && 0x10) {
             //Conversion completed
             adcResult = 0;
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ2; //Next state
          }
          break;
        case TEMPREG_ACQ2:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ3; //Next state
          }
          break;
        case TEMPREG_ACQ3:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ4; //Next state
          }
          break;
        case TEMPREG_ACQ4:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ5; //Next state
          }
          break;
        case TEMPREG_ACQ5:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ6; //Next state
          }
          break;
        case TEMPREG_ACQ6:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ7; //Next state
          }
          break;
        case TEMPREG_ACQ7:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ8; //Next state
          }
          break;
        case TEMPREG_ACQ8:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ9; //Next state
          }
          break;
        case TEMPREG_ACQ9:
          if (ADCSRA & 0x10) {
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)
             tempControllerState = TEMPREG_ACQ10; //Next state
          }
          break;
        case TEMPREG_ACQ10:
          if (ADCSRA & 0x10) {
   
             pinMode(17, OUTPUT);
             digitalWrite(17, !digitalRead(17));
      
      
             //Conversion completed
             adcResult += ADCL;
             adcResult += (ADCH << 8);
             
             //Do temperature calculation
             adcResult = adcResult / 10;    //Correct reading to 0...1023 (coming from 0...10230)
             // TODO: This should do a binary chop 
             for (i=1; i<NUMTEMPS; i++)
             {
               if (temptable[i][0] > adcResult)
               {
                 //Linear approximation of temperature between two points in table
                 currentTemperature  = temptable[i-1][1] + (adcResult - temptable[i-1][0]) 
                     * (temptable[i][1] - temptable[i-1][1])
                     / (temptable[i][0] - temptable[i-1][0]);
                 break;
               }
             }
             if (i >= NUMTEMPS) currentTemperature = temptable[i-1][1];  // Overflow: Set to last value in the table

             //PI Temperature controller          
             error = extrTargetCelsius - currentTemperature;
          
             pTerm = pGain * error;
          
             temp_iState += error;
             temp_iState = constrain(temp_iState, -30, 30);
             iTerm = iGain * temp_iState;
             
             output = pTerm + iTerm;
             output = constrain(output, 0, 156);
             
             if (extrTargetCelsius == 0) {
               OCR2B = 0;
               TCCR2A &= 0xCF;    //Disable Compare match output B so we can turn heater totally off.
               
               pinMode(E_HEATER_PIN, OUTPUT);
               digitalWrite(E_HEATER_PIN, 0);
             } else {  
               TCCR2A &= 0xEF;    //Re-enable Compare match output B.
               TCCR2A |= 0x20;
               OCR2B = output;
             }
             
             //Restart state machine:
             //ADCSRA = ADCSRA | 0x50;  //Start new conversion
             ADMUX = 0x45;    //AVCC as ref, Right adj, single ended ADC5
             DIDR0 = 0x20;    //Only ADC5 is analog, so disable digital input buffer (already done by arduino by declaring an analog input)
             ADCSRA = 0xD7;   //Enable ADC, Autotrigger dis, interrupt dis, start conversion @ 1/128 PSC (125KHz clock)            
             tempControllerState = TEMPREG_ACQ1; //Next state
          }
          break;
        default:
          tempControllerState = TEMPREG_INIT;
      }
    }
  #endif
#endif

void setup()
{
  disableTimerInterrupt();
  setupTimerInterrupt();
  interruptBlink = 0;
  pinMode(DEBUG_PIN, OUTPUT);
  debugstring[0] = 0;
  led = false;
  
  setupGcodeProcessor();
  
  ex[0] = &ex0;
#if EXTRUDER_COUNT == 2  
  ex[1] = &ex1;
#endif  
  extruder_in_use = 0; 
  
  head = 0;
  tail = 0;
  
  cdda[0] = &cdda0;
  cdda[1] = &cdda1;  
  cdda[2] = &cdda2;  
  cdda[3] = &cdda3;
  
  for(byte i = 0; i < 4; i++)
    cdda[i]->set_units(true);
  
  //setExtruder();
  
  init_process_string();
  

  Serial.begin(HOST_BAUD);
  Serial.println("start");
  
#if MOTHERBOARD == 2
    #ifndef GEN6 //Not used in generation 6 electronics
      pinMode(PS_ON_PIN, OUTPUT);  // add to run G3 as built by makerbot
      digitalWrite(PS_ON_PIN, LOW);   // ditto
    #endif
    delay(2000);    
    rs485Interface.begin(RS485_BAUD);  
#endif

  setTimer(DEFAULT_TICK);
  enableTimerInterrupt();
}

// This does a hard stop.  It disables interrupts, turns off all the motors 
// (regardless of DISABLE_X etc), and calls extruder.shutdown() for each
// extruder.  It then spins in an endless loop, never returning.  The only
// way out is to press the reset button.

void shutdown()
{
  // No more stepping
  
  disableTimerInterrupt();
  
  // Delete everything in the ring buffer
  
  cancelAndClearQueue();
  
  // LED off
  
  digitalWrite(DEBUG_PIN, 0);

  // Motors off
  
#if MOTHERBOARD > 0  
  digitalWrite(X_ENABLE_PIN, !ENABLE_ON);
  digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);
  digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);
#endif

  // Stop the extruders
  
  for(byte i = 0; i < EXTRUDER_COUNT; i++)
    ex[i]->shutdown();
  
  // Till the end of time...
  
  for(;;); 
}

//long count = 0;
//int ct1 = 0;

void loop()
{
   manageAllExtruders();
   get_and_do_command(); 
#if MOTHERBOARD == 2
   talker.tick();
#endif
}

//******************************************************************************************

// The move buffer

inline void cancelAndClearQueue()
{
	tail = head;	// clear buffer
	for(int i=0;i<BUFFER_SIZE;i++)
		cdda[i]->kill();
}

inline bool qFull()
{
  if(tail == 0)
    return head == (BUFFER_SIZE - 1);
  else
    return head == (tail - 1);
}

inline bool qEmpty()
{
   return tail == head && !cdda[tail]->active();
}

inline void qMove(const FloatPoint& p)
{
  while(qFull()) delay(WAITING_DELAY);
  byte h = head; 
  h++;
  if(h >= BUFFER_SIZE)
    h = 0;
  cdda[h]->set_target(p);
  head = h;
}

inline void dQMove()
{
  if(qEmpty())
    return;
  byte t = tail;  
  t++;
  if(t >= BUFFER_SIZE)
    t = 0;
  cdda[t]->dda_start();
  tail = t; 
}

inline void setUnits(bool u)
{
   for(byte i = 0; i < BUFFER_SIZE; i++)
     cdda[i]->set_units(u); 
}


inline void setPosition(const FloatPoint& p)
{
  where_i_am = p;  
}

void blink()
{
  led = !led;
  if(led)
      digitalWrite(DEBUG_PIN, 1);
  else
      digitalWrite(DEBUG_PIN, 0);
} 


//******************************************************************************************

// Interrupt functions

#if MOTHERBOARD == 2
  #ifdef GEN6
    // Timer 2 interrupt used for timebase of realtime temperature controller in GEN6
    void setupTempReg()
    {
      TCCR2B = 0;     //Stop timer in case of running
        
      TCCR2A = 0x23;  //OC2A disable; FastPWM noninverting; FastPWM mode 7
      OCR2A = 156;    //Period is ~10ms
      OCR2B = 0;      //Duty Cycle for heater pin is 0 (startup)
      TIMSK2 = 0x01;  //Enable overflow interrupt
      TCCR2B = 0x0F;  //1/1024 prescaler, start
    }
  #endif
#endif

void setupTimerInterrupt()
{
	//clear the registers
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	
	//waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11); 
	TCCR1A &= ~(1<<WGM10);

	//output mode = 00 (disconnected)
	TCCR1A &= ~(1<<COM1A1); 
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A &= ~(1<<COM1B1); 
	TCCR1A &= ~(1<<COM1B0);

	//start off with a slow frequency.
	setTimerResolution(4);
	setTimerCeiling(65535);
}

void setTimerResolution(byte r)
{
	//here's how you figure out the tick size:
	// 1000000 / ((16000000 / prescaler))
	// 1000000 = microseconds in 1 second
	// 16000000 = cycles in 1 second
	// prescaler = your prescaler

	// no prescaler == 0.0625 usec tick
	if (r == 0)
	{
		// 001 = clk/1
		TCCR1B &= ~(1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B |=  (1<<CS10);
	}	
	// prescale of /8 == 0.5 usec tick
	else if (r == 1)
	{
		// 010 = clk/8
		TCCR1B &= ~(1<<CS12);
		TCCR1B |=  (1<<CS11);
		TCCR1B &= ~(1<<CS10);
	}
	// prescale of /64 == 4 usec tick
	else if (r == 2)
	{
		// 011 = clk/64
		TCCR1B &= ~(1<<CS12);
		TCCR1B |=  (1<<CS11);
		TCCR1B |=  (1<<CS10);
	}
	// prescale of /256 == 16 usec tick
	else if (r == 3)
	{
		// 100 = clk/256
		TCCR1B |=  (1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B &= ~(1<<CS10);
	}
	// prescale of /1024 == 64 usec tick
	else
	{
		// 101 = clk/1024
		TCCR1B |=  (1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B |=  (1<<CS10);
	}
}

unsigned int getTimerCeiling(const long& delay)
{
	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs)
	if (delay <= 65535L)
		return (delay & 0xffff);
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs)
	else if (delay <= 524280L)
		return ((delay / 8) & 0xffff);
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs)
	else if (delay <= 4194240L)
		return ((delay / 64) & 0xffff);
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs)
	else if (delay <= 16776960L)
		return ((delay / 256) & 0xffff);
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs)
	else if (delay <= 67107840L)
		return ((delay / 1024) & 0xffff);
	//its really slow... hopefully we can just get by with super slow.
	else
		return 65535;
}

byte getTimerResolution(const long& delay)
{
	// these also represent frequency: 1000000 / delay / 2 = frequency in hz.
	
	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs (4 millisecond max))
	// range: 8Mhz max - 122hz min
	if (delay <= 65535L)
		return 0;
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs (32 millisecond max))
	// range:1Mhz max - 15.26hz min
	else if (delay <= 524280L)
		return 1;
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs (0.26 seconds max))
	// range: 125Khz max - 1.9hz min
	else if (delay <= 4194240L)
		return 2;
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs (1.04 seconds max))
	// range: 31.25Khz max - 0.475hz min
	else if (delay <= 16776960L)
		return 3;
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs (4.19 seconds max))
	// range: 7.812Khz max - 0.119hz min
	else if (delay <= 67107840L)
		return 4;
	//its really slow... hopefully we can just get by with super slow.
	else
		return 4;
}


// Depending on how much work the interrupt function has to do, this is
// pretty accurate between 10 us and 0.1 s.  At fast speeds, the time
// taken in the interrupt function becomes significant, of course.

// Note - it is up to the user to call enableTimerInterrupt() after a call
// to this function.

inline void setTimer(long delay)
{
	// delay is the delay between steps in microsecond ticks.
	//
	// we break it into 5 different resolutions based on the delay. 
	// then we set the resolution based on the size of the delay.
	// we also then calculate the timer ceiling required. (ie what the counter counts to)
	// the result is the timer counts up to the appropriate time and then fires an interrupt.

        // Actual ticks are 0.0625 us, so multiply delay by 16
        
        delay <<= 4;
        
	setTimerCeiling(getTimerCeiling(delay));
	setTimerResolution(getTimerResolution(delay));
}


void delayMicrosecondsInterruptible(unsigned int us)
{
  // for a one-microsecond delay, simply return.  the overhead
  // of the function call yields a delay of approximately 1 1/8 us.
  if (--us == 0)
    return;

  // the following loop takes a quarter of a microsecond (4 cycles)
  // per iteration, so execute it four times for each microsecond of
  // delay requested.
  us <<= 2;

  // account for the time taken in the preceeding commands.
  us -= 2;

  // busy wait
  __asm__ __volatile__ ("1: sbiw %0,1" "\n\t" // 2 cycles
"brne 1b" : 
  "=w" (us) : 
  "0" (us) // 2 cycles
    );
}

