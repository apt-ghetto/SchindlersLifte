/******************************************************************************
* Program:   Lift simulation Basic Structure
* Filename:  LiftSimulator_Task_ABasic
* Author:    Werner Odermatt
* Version:   1.0
* Date:	     09.12.2016
*
* Development flow(Version, Date, Author, Development step, Time):
* 1.0   09.12.16   WOd   Basic structure created
********************************************************************************
*
* Using: C-Training, M121, M242
*
* Description:
* Basic structure for lift simulation.
*
* Precondition:  -
*
* Postcondition: -
*
* Required Libraries:
* - avr/io.h
* - LiftLibrary.h
*
* Created Functions:
* - ConvertButtonTypeToLiftPosType()
* - CheckKeyEvent()
* - UpdateDisplay()
*
* Copyright (c) 2016 by W.Odermatt, CH-6340 Baar
*******************************************************************************/

/*** OWN DATA TYPES ***********************************************************/
typedef enum {Uninitialized = 0, Waiting, CloseDoor, MoveLift, OpenDoor, Trouble}
StateMachineType;

/*** CONSTANTS ****************************************************************/


/*** INCLUDE FILES ************************************************************/
#include "LiftLibrary.h" // lift model library



/*** GLOBAL Variablen *********************************************************/
StateMachineType  state = Uninitialized;
LiftPosType       requestedElevatorPosition = None;
LiftPosType       currentElevatorState = None;
DirectionType     elevatorDirection = Down;


/*******************************************************************************
***  PRIVATE FUNCTIONS  ********************************************************
*******************************************************************************/
// Convert ButtonType to LiftPosType
LiftPosType ConvertButtonTypeToLiftPosType (ButtonType button);

// Check if buttons are pressed
ButtonType CheckKeyEvent ();

// Update the 7-Seg. display
void UpdateDisplay (LiftPosType elevatorState);


/*******************************************************************************
*** MAIN PROGRAM
*******************************************************************************/
int main(void)
{
  InitializePorts();  // Initialization of ports
  InitializeStart();  // Set start state of the system

  // Endless loop
  while(1)
  {
    // Handling state machine
    switch (state)
    {
      case Uninitialized:
      {
        // Lift position calibration to ground floor (Floor0)

        break;
      }


      case Waiting:
      {
        // Waiting for new floor request

        break;
      }


      case CloseDoor:
      {
        // Close the door and wait until the door is closed

        break;
      }


      case MoveLift:
      {
        // Move cabin to the requested floor

        break;
      }


      case OpenDoor:
      {
        // Open the door and wait still the door is open completely

        break;
      }


      case Trouble:
      {
        // Fault condition is not treated
        break;
      }
    }

    UpdateDisplay(currentElevatorState);  // Update the 7-Seg. display (lift)
    SetOutput();               // Send the calculated output values to the ports
  }

  return (0);
}


/*******************************************************************************
***  PRIVATE FUNCTIONs *********************************************************
*******************************************************************************/
// Convert ButtonType to LiftPosType
LiftPosType ConvertButtonTypeToLiftPosType (ButtonType button)
{
  LiftPosType retVal = None;

  switch (button)
  {
    case LiftButton_F0:
    case FloorButton_F0:
    {
      retVal = Floor0;
      break;
    }
    case LiftButton_F1:
    case FloorButton_F1:
    {
      retVal = Floor1;
      break;
    }
    case LiftButton_F2:
    case FloorButton_F2:
    {
      retVal = Floor2;
      break;
    }
    case LiftButton_F3:
    case FloorButton_F3:
    {
      retVal = Floor3;
      break;
    }
    default:
    {
      //retVal = None;
      break;
    }
  }

  return retVal;
}

// Check if buttons are pressed
ButtonType CheckKeyEvent ()
{
  ButtonType retVal = EmergencyButton;

  for (ButtonType key = FloorButton_F3; ((key >= LiftButton_F0) && (retVal == EmergencyButton)); key>>=1)
  {
    if (ReadKeyEvent(key) == Pressed)
    {
      retVal = key;
    }
  }
  return retVal;
}

// Update the 7-Seg. display
void UpdateDisplay (LiftPosType elevatorState)
{
  switch (elevatorState)
  {
    case Floor0:
    case Floor1:
    case Floor2:
    case Floor3:
    case Error:
    case Test:
    {
      SetDisplay(elevatorState);
      break;
    }
    default:
    {
      break;
    }
  }
}



