#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*
 *  This is the configuration file for the RepRap Motherboard microcontroller.
 *  Set values in it to match your RepRap machine.
 *  
 *  Lines in here with a 
 *
 *        // *RO
 *
 *  Comment at the end (read-only) should probably only be changed if you really
 *  know what you are doing...
 */

// Here are the Motherboard codes; set MOTHERBOARD to the right one
// A standard Mendel is MOTHERBOARD 2

// (Arduino: 0 - no longer in use)
// Sanguino or RepRap Motherboard with direct drive extruders: 1
// RepRap Motherboard with RS485 extruders: 2
// Arduino Mega: 3
// Generation 6 electronics: 2, also define GEN6

#define MOTHERBOARD 2
#define GEN6

//*********************************************************************************************

// These settings are mainly for Darwin

#if MOTHERBOARD == 1
  
  // Comment out this if you are using a thermocouple
  #define USE_THERMISTOR
  
  // Set to 1 if enable pins are inverting
  // For RepRap stepper boards version 1.x the enable pins are *not* inverting.
  // For RepRap stepper boards version 2.x and above the enable pins are inverting.
  #define INVERT_ENABLE_PINS 0
  
  // Set to one if the axis opto-sensor outputs inverting (ie: 1 means open, 0 means closed)
  // RepRap opto endstops with H21LOI sensors are not inverting; ones with H21LOB
  // are inverting.
  
  #define X_ENDSTOP_INVERTING false
  #define Y_ENDSTOP_INVERTING false
  #define Z_ENDSTOP_INVERTING false

#endif

//**********************************************************************************************

// These settings are mainly for a standard Mendel

#if MOTHERBOARD == 2
  
  // Comment out the next line if you are running a Darwin with a MOTHERBOARD > 1  
  
  #define MENDEL 1
  
  // Set to 1 if enable pins are inverting
  // For RepRap stepper boards version 1.x the enable pins are *not* inverting.
  // For RepRap stepper boards version 2.x and above the enable pins are inverting.
  #define INVERT_ENABLE_PINS 1
  
  // Set to one if the axis opto-sensor outputs inverting (ie: 1 means open, 0 means closed)
  // RepRap opto endstops with H21LOI sensors are not inverting; ones with H21LOB
  // are inverting.
  // New Endstops from Mendel-Parts are non-inverting (TCST2103), (so false below), but if you use old endstop
  // Those were shipped before 17Dec 2010, you need true, this is firmware version is for old type opto, so true
  
  #define X_ENDSTOP_INVERTING true
  #define Y_ENDSTOP_INVERTING true
  #define Z_ENDSTOP_INVERTING true
  
  // Does your machine have the older standard slotted pullyes (needing lots of motor shaft filing)
  // or the newer pulleys with grub screws?  Uncomment one of the two next lines.
  //#define SLOTTED_PULLEYS
  #define GRUB_PULLEYS
  
  #define MY_NAME 'H'           // Byte representing the name of this device
  #define E0_NAME '0'           // Byte representing the name of extruder 0
  #define E1_NAME '1'           // Byte representing the name of extruder 1
  
  #define RS485_MASTER  1       // *RO
  
  #ifdef GEN6
    #define USE_THERMISTOR
    
    #define TEMPREG_INIT 0
    #define TEMPREG_ACQ1 1
    #define TEMPREG_ACQ2 2
    #define TEMPREG_ACQ3 3
    #define TEMPREG_ACQ4 4
    #define TEMPREG_ACQ5 5
    #define TEMPREG_ACQ6 6
    #define TEMPREG_ACQ7 7
    #define TEMPREG_ACQ8 8
    #define TEMPREG_ACQ9 9
    #define TEMPREG_ACQ10 10
    
    // The temperature routines get called approx every TEMP_TIME_INTERVAL msec
    #define TEMP_TIME_INTERVAL 500
  #endif

#endif

//**********************************************************************************************

// These settings are mainly for a Mendel with an Arduino Mega controller

#if MOTHERBOARD == 3
  #define MENDEL 1
  
  // Comment out this if you are using a thermocouple
  #define USE_THERMISTOR
  
  // Set to 1 if enable pins are inverting
  // For RepRap stepper boards version 1.x the enable pins are *not* inverting.
  // For RepRap stepper boards version 2.x and above the enable pins are inverting.
  #define INVERT_ENABLE_PINS 1
  
  // Set to one if the axis opto-sensor outputs inverting (ie: 1 means open, 0 means closed)
  // RepRap opto endstops with H21LOI sensors are not inverting; ones with H21LOB
  // are inverting.
  
  #define X_ENDSTOP_INVERTING true
  #define Y_ENDSTOP_INVERTING true
  #define Z_ENDSTOP_INVERTING true
  
  // Does your machine have the older standard slotted pullyes (needing lots of motor shaft filing)
  // or the newer pulleys with grub screws?  Uncomment one of the two next lines.
  //#define SLOTTED_PULLEYS
  #define GRUB_PULLEYS
#endif

//**********************************************************************************************

// The speed at which to talk with the host computer; default is 19200
#define HOST_BAUD 19200 // *RO

// The number of real extruders
#define EXTRUDER_COUNT 2

// Set 1s where you have endstops; 0s where you don't
// Both Darwin and Mendel have MIN endstops, but not MAX ones.
#define ENDSTOPS_MIN_ENABLED 1
#define ENDSTOPS_MAX_ENABLED 0

// The width of Henry VIII's thumb (or something).
#define INCHES_TO_MM 25.4 // *RO

// The number of mm below which distances are insignificant (one tenth the
// resolution of the machine is the default value).
#define SMALL_DISTANCE 0.01 // *RO

// Useful to have its square
#define SMALL_DISTANCE2 (SMALL_DISTANCE*SMALL_DISTANCE) // *RO

#ifdef MENDEL
  #ifdef SLOTTED_PULLEYS
    // define the XYZ parameters of Mendel - standard pulleys
    
    #define X_STEPS_PER_MM   10.047
    #define X_STEPS_PER_INCH (X_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_X_DIR 0
    
    #define Y_STEPS_PER_MM   10.047
    #define Y_STEPS_PER_INCH (Y_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_Y_DIR 0
    
    #define Z_STEPS_PER_MM   833.398
    #define Z_STEPS_PER_INCH (Z_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_Z_DIR 0
  #endif
  
  #ifdef GRUB_PULLEYS
  // define the XYZ parameters of Mendel - grub-screw pulleys
    
    #define X_STEPS_PER_MM   40.000   // Edited 20100715 @ EJE 10.047 // Edited by CamielG @ 20101216 - 40.000
    #define X_STEPS_PER_INCH (X_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_X_DIR 0
    
    #define Y_STEPS_PER_MM   40.000   // Edited 20100715 @ EJE 10.047 // Edited by CamielG @ 20101216 - 40.000
    #define Y_STEPS_PER_INCH (Y_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_Y_DIR 1
    
    #define Z_STEPS_PER_MM   3333.592  // 833.398
    #define Z_STEPS_PER_INCH (Z_STEPS_PER_MM*INCHES_TO_MM) // *RO
    #define INVERT_Z_DIR 0
  #endif
#endif

#if MOTHERBOARD == 1
// This is for Darwin.
  #define X_STEPS_PER_MM   7.99735
  #define X_STEPS_PER_INCH (X_STEPS_PER_MM*INCHES_TO_MM) // *RO
  #define INVERT_X_DIR 0
  
  #define Y_STEPS_PER_MM   7.99735
  #define Y_STEPS_PER_INCH (Y_STEPS_PER_MM*INCHES_TO_MM) // *RO
  #define INVERT_Y_DIR 0
  
  #define Z_STEPS_PER_MM   320
  #define Z_STEPS_PER_INCH (Z_STEPS_PER_MM*INCHES_TO_MM) // *RO
  #define INVERT_Z_DIR 0
#endif

// For when we have a stepper-driven extruder
// E_STEPS_PER_MM is the number of steps needed to 
// extrude 1mm out of the nozzle.  E0 for extruder 0;
// E1 for extruder 1, and so on.

//#define E_STEPS_PER_MM   0.9 //0.706   // NEMA 17 extruder 5mm diameter drive - empirically adjusted

//#define E_STEPS_PER_MM   2.2     // NEMA 14 geared extruder 8mm diameter drive
#define E0_STEPS_PER_MM   20.2     //Miocrostepping! Edited 20100715 @ EJE. 2.0       
                                   // NEMA 17 55/11 geared extruder 7mm diameter drive
                                   // 1/(2* 1/8 * 1/200 * 11/55 * 7*pi * (pi*1,5^2)/(pi*0,25^2))
#define E1_STEPS_PER_MM   2.0      // NEMA 17 55/11 geared extruder 8mm diameter drive

//#define E0_STEPS_PER_INCH (E_STEPS_PER_MM*INCHES_TO_MM) // *RO

//our maximum feedrates
#define FAST_XY_FEEDRATE 3000.0
#define FAST_Z_FEEDRATE  50.0

// Data for acceleration calculations
// Comment out the next line to turn accelerations off
//#define ACCELERATION_ON
#define SLOW_XY_FEEDRATE 1000.0 // Speed from which to start accelerating
#define SLOW_Z_FEEDRATE 20


#if INVERT_ENABLE_PINS == 1  // *RO
  #define ENABLE_ON LOW        // *RO
#else                        // *RO
  #define ENABLE_ON HIGH       // *RO
#endif                       // *RO

// Set these to 1 to disable an axis when it's not being used,
// and for the extruder.  Usually only Z is disabled when not in
// use.  You will probably find that disabling the others (i.e.
// powering down the steppers that drive them) when the ends of
// movements are reached causes poor-quality builds.  (Inertia
// causes overshoot if the motors are not left powered up.)

#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 1
#define DISABLE_E 0

// The number of 5-second intervals to wait at the target temperature for things to stabilise.
// Too short, and the extruder will jam as only part of it will be hot enough.
// Too long and the melt will extend too far up the insulating tube.
// Default value: 10

#define WAIT_AT_TEMPERATURE 10

//our command string length

#define COMMAND_SIZE 128 // *RO

// The size of the movement buffer

#define BUFFER_SIZE 4 // *RO

// Number of microseconds between timer interrupts when no movement
// is happening

#define DEFAULT_TICK (long)1000 // *RO

// What delay() value to use when waiting for things to free up in milliseconds

#define WAITING_DELAY 1 // *RO

//******************************************************************************

// You probably only want to edit things below this line if you really really
// know what you are doing...
extern char debugstring[];

void delayMicrosecondsInterruptible(unsigned int us);

// Inline interrupt control functions
inline void enableTimerInterrupt() 
{
  TIMSK1 |= (1<<OCIE1A);
}
	
inline void disableTimerInterrupt() 
{
  TIMSK1 &= ~(1<<OCIE1A);
}
        
inline void setTimerCeiling(unsigned int c) 
{
  OCR1A = c;
}

inline void resetTimer()
{
  TCNT2 = 0;
}

#endif

//*************************************************************************

#if 0
  // Green machine:
  //#define ENDSTOPS_INVERTING 0
  
  // parameters for the Bath U. mendel prototype
  #define X_STEPS_PER_MM   13.333333
  #define X_STEPS_PER_INCH (X_STEPS_PER_MM*INCHES_TO_MM)
  #define INVERT_X_DIR 0
  
  #define Y_STEPS_PER_MM   13.333333
  #define Y_STEPS_PER_INCH (Y_STEPS_PER_MM*INCHES_TO_MM)
  #define INVERT_Y_DIR 0
  
  // Green machine:
  #define Z_STEPS_PER_MM   944.88
  // Fat Z cog machine:
  //#define Z_STEPS_PER_MM   558.864
  // Standard Mendel:
  //#define Z_STEPS_PER_MM   833.398
  #define Z_STEPS_PER_INCH (Z_STEPS_PER_MM*INCHES_TO_MM)
  #define INVERT_Z_DIR 0
#endif
