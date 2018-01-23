/******************************************************************************
* Program:   Lift simulation Basic Structure
* Filename:  LiftSimulator_Task_BBasic
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

/*** OWN DEFINES **************************************************************/
#define BUFFER_SIZE     3
#define BUFFER_SUCCESS  0
#define BUFFER_FAIL     1
#define FALSE			0
#define TRUE			1


/*** INCLUDE FILES ************************************************************/
#include "LiftLibrary.h"		// lift model library


/*** OWN DATA TYPES ***********************************************************/
typedef enum {Uninitialized = 0, Waiting, CloseDoor, MoveLift, OpenDoor, Trouble}
StateMachineType;


/*** GLOBAL Variablen *********************************************************/
StateMachineType  state = Uninitialized;
LiftPosType       requestedElevatorPosition = None;
LiftPosType       currentElevatorState = None;
DirectionType     elevatorDirection = Down;

// ringbuffer for stored requests (floors)
LiftPosType		  callBuffer[BUFFER_SIZE];

// read and write pointers for buffer
// initialized with beginning of buffer
LiftPosType		  *readPointer = callBuffer;
LiftPosType		  *writePointer = callBuffer;

// flag for inverted state of buffer
uint8_t			  invert = FALSE;


/*******************************************************************************
***  PRIVATE FUNCTIONS  ********************************************************
*******************************************************************************/
// Convert ButtonType to LiftPosType
LiftPosType ConvertButtonTypeToLiftPosType (ButtonType button);

// Check if buttons are pressed
ButtonType CheckKeyEvent ();

// Update the 7-Seg. display
void UpdateDisplay (LiftPosType elevatorState);

// Add a requested floor to the buffer
uint8_t AddRequestToBuffer(LiftPosType floorRequest);

// Get a request from the buffer if there is one
uint8_t GetRequestFromBuffer();


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
				if (ReadElevatorState() != Floor0)
				{
					CalibrateElevatorPosition();
				}
				else
				{
					state = OpenDoor;
					currentElevatorState = ReadElevatorState();
				}
				break;
			}


			case Waiting:
			{
				// check for saved calls in buffer
				if (!GetRequestFromBuffer())
				{
					// request was found
					state = CloseDoor;
				}

				break;
			}


			case CloseDoor:
			{
				// Close the door and wait until the door is closed
				if (ReadDoorState(currentElevatorState) != Closed)
				{
					SetDoorState(Closed, currentElevatorState);
				}
				else
				{
					state = MoveLift;
				}

				break;
			}


			case MoveLift:
			{
				// Move cabin to the requested floor
				if (currentElevatorState != requestedElevatorPosition)
				{
					MoveElevator(elevatorDirection, Fast);
				}
				else
				{
					state = OpenDoor;
				}
				
				break;
			}


			case OpenDoor:
			{
				// Open the door and wait still the door is open completely
				SetDoorState(Open, currentElevatorState);
				if (ReadDoorState(currentElevatorState) == Open)
				{
					state = Waiting;
					ClrIndicatorFloorState(currentElevatorState);
					ClrIndicatorElevatorState(currentElevatorState);
				}
				break;
			}


			case Trouble:
			{
				// Fault condition is not treated
				break;
			}
		}

		// do always
		UpdateDisplay(currentElevatorState);  // Update the 7-Seg. display (lift)
		currentElevatorState = ReadElevatorState();
		SetOutput();               // Send the calculated output values to the ports
		
		// check if button is pressed
		ButtonType newKey = CheckKeyEvent();
		LiftPosType pressedFloor = ConvertButtonTypeToLiftPosType(newKey);
		
		// if a button is pressed, check if it is a floor-request
		// and if it's not the current floor
		if (pressedFloor <= 3 && pressedFloor != currentElevatorState)
		{
			// if call is saved to buffer, set indicators
			if (!AddRequestToBuffer(pressedFloor))
			{
				newKey < 16 ? SetIndicatorElevatorState(pressedFloor)
				: SetIndicatorFloorState(pressedFloor);
			}
		}
	}

	return (0);
}


/*******************************************************************************
***  PRIVATE FUNCTIONs *********************************************************
*******************************************************************************/

// Add a Request to the circular buffer
uint8_t AddRequestToBuffer(LiftPosType pressedFloor)
{
	// return fail if buffer is full
	if (readPointer == writePointer && invert == TRUE) {
		return BUFFER_FAIL;
	}

	// return success if requested floor is the current destination
	// don't save request again but toggle indicator in main function
	if (pressedFloor == requestedElevatorPosition)
	{
		return BUFFER_SUCCESS;
	}

	// check if the requested floor is already in the buffer
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		// return success to toggle indicators but don't save in buffer again
		if (callBuffer[i] == pressedFloor)
		{
			return BUFFER_SUCCESS;
		}
	}

	// save request to buffer
	*writePointer = pressedFloor;

	// move write pointer to next position
	writePointer++;

	// check if write pointer is at the end of the buffer
	// if so, set to the beginning again
	if ( writePointer > &callBuffer[BUFFER_SIZE - 1] ) {
		writePointer = callBuffer;
		invert = TRUE;
	}

	return BUFFER_SUCCESS;
}

// Get a Button from the circular buffer
uint8_t GetRequestFromBuffer()
{
	// return fail if no calls are in buffer
	if (readPointer == writePointer && invert == FALSE) {
		return BUFFER_FAIL;
	}
	
	// read floor from buffer
	requestedElevatorPosition = *readPointer;

	// check elevator direction
	elevatorDirection = requestedElevatorPosition > currentElevatorState;

	// delete the request from the buffer
	*readPointer = None;
	
	// increment read position
	readPointer++;

	// reset read position to 0 if end of buffer is reached
	if (readPointer > &callBuffer[BUFFER_SIZE - 1]) {
		readPointer = callBuffer;
		invert = FALSE;
	}
	
	return BUFFER_SUCCESS;
}

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



