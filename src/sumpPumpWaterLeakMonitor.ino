/*
 * Project sumpPumpWaterLeakMonitor
 * Description: monitor a sump pump and water leaks
 * Author: Daniel Adams/Steve McKee/Gustavo Gonnet
 * Date: May 27th 2019
 *
 * Vertical Water Leak Sensor Detector for Home Security Warning Signal System
 *   https://www.ebay.com/itm/DC-12V-Water-Leak-Sensor-Detector-for-Home-Security-Warning-Alert-Alarm-EC/362538089184?epid=628768485&hash=item5468f43ae0:g:CfQAAOSw5wFcIFGx&autorefresh=true
 * Flat water leak sensor
 *   https://www.ebay.com/itm/5-Pcs-Water-Leak-Flood-Alarm-Sensor-w-Leads-/113535583234
 * 
 */

// Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// This is a human-readable summary of (and not a substitute for) the license.
// Disclaimer
//
// You are free to:
// Share — copy and redistribute the material in any medium or format
// Adapt — remix, transform, and build upon the material
// The licensor cannot revoke these freedoms as long as you follow the license terms.
//
// Under the following terms:
// Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
// NonCommercial — You may not use the material for commercial purposes.
// ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
// No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
//
// Notices:
// You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
// No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
//
// github: https://github.com/gusgonnet
// hackster: https://www.hackster.io/gusgonnet
//
// Free for personal use.
//
// https://creativecommons.org/licenses/by-nc-sa/4.0/

// #include "elapsedMillis.h"
// #include "AnalogSmooth.h"
#include "FiniteStateMachine.h"

#define APP_NAME "sumpPumpWaterLeakMonitor"
#define VERSION "Version 0.01"

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
*******************************************************************************/
/*******************************************************************************
 * TODO:
   * define where to plug water sensor
   * define where to plug sump pump level sensors
   * add software watchdog
   * add sump pump level monitor sensors
*******************************************************************************/

// enable the user code (our program below) to run in parallel with cloud connectivity code
// source: https://docs.particle.io/reference/firmware/photon/#system-thread
SYSTEM_THREAD(ENABLED);

#define WATER_LEAK_SENSOR A3
unsigned int integratorWaterLeakSensor = 0;
int waterLeakSensor = 0;

#define SUMP_PUMP_SENSOR_HIGH D0
unsigned int integratorSumpPumpSensorHigh = 0;
int sumpPumpSensorHigh = 0;

#define SUMP_PUMP_SENSOR_VERY_HIGH D1
unsigned int integratorSumpPumpSensorVeryHigh = 0;
int sumpPumpSensorVeryHigh = 0;

// FSM declaration
#define STATE_OK "Sensor OK"
#define STATE_ALARM "Sensor Alarm"
#define ALARM_MIN 30000 // min amount of time to stay in alarm before coming back to normal

State waterLeakOkState = State(waterLeakOkEnterFunction, waterLeakOkUpdateFunction, waterLeakOkExitFunction);
State waterLeakAlarmState = State(waterLeakAlarmEnterFunction, waterLeakAlarmUpdateFunction, waterLeakAlarmExitFunction);
FSM waterLeakStateMachine = FSM(waterLeakOkState);
String waterLeakState = STATE_OK;

/* The following parameters tune the algorithm to fit the particular
application.  The example numbers are for a case where a computer samples a
mechanical contact 10 times a second and a half-second integration time is
used to remove bounce.  Note: DEBOUNCE_TIME is in seconds and SAMPLE_FREQUENCY
is in Hertz */
// source: http://www.kennethkuhn.com/electronics/debounce.c
Timer debounceInputsTimer(20, debounceInputs);
#define DEBOUNCE_TIME 0.1
#define SAMPLE_FREQUENCY 50
#define MAXIMUM (DEBOUNCE_TIME * SAMPLE_FREQUENCY)

bool testPublishFlag = false;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{

  // cloud functions
  // Up to 15 cloud functions may be registered and each function name is limited to
  // a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-function-
  Particle.function("testPublish", testPublish);

  // cloud variables
  // Up to 20 cloud variables may be registered and each variable name is limited to
  // a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-variable-
  Particle.variable("waterLeakState", waterLeakState);

  pinMode(WATER_LEAK_SENSOR, INPUT);

  debounceInputsTimer.start();

  // publish start up message with firmware version
  Particle.publish(APP_NAME, VERSION, PRIVATE | WITH_ACK);
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{

  waterLeakStateMachine.update();

  // check if the testPublish cloud function was called
  checkTestPublishFlag();
}

/*******************************************************************************
 * Function Name  : checkTestPublishFlag
 * Description    : test a publish message if the testPublish() function was called
 *******************************************************************************/
void checkTestPublishFlag()
{
  if (testPublishFlag)
  {
    testPublishFlag = false;
    Particle.publish("TEST", "test message 123", PRIVATE | WITH_ACK);
  }
}

/*******************************************************************************
 * Function Name  : testPublish
 * Description    : test a publish message
                    we just set a flag here that will trigger the action in the loop()
                    it is considered bad practice to call publish from a cloud function
 *******************************************************************************/
int testPublish(String dummy)
{
  testPublishFlag = true;
  return 0;
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 WATER LEAK sensor
*******************************************************************************/
void waterLeakOkEnterFunction()
{
  waterLeakSetState(STATE_OK);
}
void waterLeakOkUpdateFunction()
{
  if (waterLeakSensor == 1)
  {
    waterLeakStateMachine.transitionTo(waterLeakAlarmState);
  }
}
void waterLeakOkExitFunction()
{
}
void waterLeakAlarmEnterFunction()
{
  Particle.publish("ALARM", "Alarm on WATER LEAK sensor", PRIVATE | WITH_ACK);
  waterLeakSetState(STATE_ALARM);
}
void waterLeakAlarmUpdateFunction()
{
  // stay here a minimum time
  if (waterLeakStateMachine.timeInCurrentState() < ALARM_MIN)
  {
    return;
  }
  // sensor went back to normal
  if (waterLeakSensor == 0)
  {
    waterLeakStateMachine.transitionTo(waterLeakOkState);
  }
}
void waterLeakAlarmExitFunction()
{
  Particle.publish("OK", "WATER LEAK sensor OK", PRIVATE | WITH_ACK);
}

/*******************************************************************************
 * Function Name  : waterLeakSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void waterLeakSetState(String newState)
{
  waterLeakState = newState;
  Particle.publish("FSM", "WATER LEAK fsm entering " + newState + " state", PRIVATE | WITH_ACK);
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 HELPER FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : debounceInputs
 * Description    : debounces the contact sensors, triggered by a timer with similar name
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceInputs()
{
  debounceWaterSensor();
  debounceSumpPumpSensorHigh();
  debounceSumpPumpSensorVeryHigh();
}

/*******************************************************************************
 * Function Name  :     debounceWaterSensor
 * Description    : debounces the water leak sensor
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceWaterSensor()
{
  // Step 1: Update the integrator based on the input signal.  Note that the
  // integrator follows the input, decreasing or increasing towards the limits as
  // determined by the input state (0 or 1).
  if (digitalRead(WATER_LEAK_SENSOR) == LOW)
  {
    if (integratorWaterLeakSensor > 0)
      integratorWaterLeakSensor--;
  }
  else if (integratorWaterLeakSensor < MAXIMUM)
    integratorWaterLeakSensor++;

  // Step 2: Update the output state based on the integrator.  Note that the
  // output will only change states if the integrator has reached a limit, either
  // 0 or MAXIMUM.
  if (integratorWaterLeakSensor == 0)
    waterLeakSensor = 0;
  else if (integratorWaterLeakSensor >= MAXIMUM)
  {
    waterLeakSensor = 1;
    integratorWaterLeakSensor = MAXIMUM; /* defensive code if integrator got corrupted */
  }
}

/*******************************************************************************
 * Function Name  :     debounceSumpPumpSensorHigh
 * Description    : debounces the sump pump sensor high level
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceSumpPumpSensorHigh()
{
  // Step 1: Update the integrator based on the input signal.  Note that the
  // integrator follows the input, decreasing or increasing towards the limits as
  // determined by the input state (0 or 1).
  if (digitalRead(SUMP_PUMP_SENSOR_HIGH) == LOW)
  {
    if (integratorSumpPumpSensorHigh > 0)
      integratorSumpPumpSensorHigh--;
  }
  else if (integratorSumpPumpSensorHigh < MAXIMUM)
    integratorSumpPumpSensorHigh++;

  // Step 2: Update the output state based on the integrator.  Note that the
  // output will only change states if the integrator has reached a limit, either
  // 0 or MAXIMUM.
  if (integratorSumpPumpSensorHigh == 0)
    sumpPumpSensorHigh = 0;
  else if (integratorSumpPumpSensorHigh >= MAXIMUM)
  {
    sumpPumpSensorHigh = 1;
    integratorSumpPumpSensorHigh = MAXIMUM; /* defensive code if integrator got corrupted */
  }
}

/*******************************************************************************
 * Function Name  :     debounceSumpPumpSensorVeryHigh
 * Description    : debounces the sump pump sensor very high level
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceSumpPumpSensorVeryHigh()
{
  // Step 1: Update the integrator based on the input signal.  Note that the
  // integrator follows the input, decreasing or increasing towards the limits as
  // determined by the input state (0 or 1).
  if (digitalRead(SUMP_PUMP_SENSOR_VERY_HIGH) == LOW)
  {
    if (integratorSumpPumpSensorVeryHigh > 0)
      integratorSumpPumpSensorVeryHigh--;
  }
  else if (integratorSumpPumpSensorVeryHigh < MAXIMUM)
    integratorSumpPumpSensorVeryHigh++;

  // Step 2: Update the output state based on the integrator.  Note that the
  // output will only change states if the integrator has reached a limit, either
  // 0 or MAXIMUM.
  if (integratorSumpPumpSensorVeryHigh == 0)
    sumpPumpSensorVeryHigh = 0;
  else if (integratorSumpPumpSensorVeryHigh >= MAXIMUM)
  {
    sumpPumpSensorVeryHigh = 1;
    integratorSumpPumpSensorVeryHigh = MAXIMUM; /* defensive code if integrator got corrupted */
  }
}
