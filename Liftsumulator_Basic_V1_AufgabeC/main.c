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

/*** OWN DATA TYPES ***********************************************************/
typedef enum {Uninitialized = 0, Waiting, CloseDoor, MoveLift, OpenDoor, Trouble}
StateMachineType;

/*** INCLUDE FILES ************************************************************/
#include "LiftLibrary.h" // lift model library

/*** CONSTANTS ****************************************************************/
const uint8_t STEPS = 16;

/*** GLOBAL Variablen *********************************************************/
StateMachineType    state = Uninitialized;
LiftPosType         requestedElevatorPosition = None;
LiftPosType         currentElevatorState = None;
DirectionType       elevatorDirection = Down;
uint8_t             currLiftPos = 0;
uint8_t             requLiftPos = 0;

struct ringBuffer {
    ButtonType data[BUFFER_SIZE];
    uint8_t read;
    uint8_t write;
} callBuffer = { {}, 0, 0 };


/*******************************************************************************
***  PRIVATE FUNCTIONS  ********************************************************
*******************************************************************************/
// Convert ButtonType to LiftPosType
LiftPosType ConvertButtonTypeToLiftPosType (ButtonType button);

// Check if buttons are pressed
ButtonType CheckKeyEvent ();

// Update the 7-Seg. display
void UpdateDisplay (LiftPosType elevatorState);

uint8_t AddButtonToBuffer(ButtonType button);
uint8_t GetButtonFromBuffer(ButtonType *button);
SpeedType GetSpeedType();


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
        // do always
        UpdateDisplay(currentElevatorState);  // Update the 7-Seg. display (lift)
        currentElevatorState = ReadElevatorState();
        SetOutput();               // Send the calculated output values to the ports

        ButtonType newKey = CheckKeyEvent();

        if (EmergencyButton != newKey) {
            AddButtonToBuffer(newKey);
        }

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
                ButtonType key;
                // Waiting for new floor request
                if (!GetButtonFromBuffer(&key))
                {
                    // button was pressed
                    requestedElevatorPosition = ConvertButtonTypeToLiftPosType(key);
                    int result = currentElevatorState - requestedElevatorPosition;
                    if (result != 0)
                    {
                        if (result < 0) {
                            elevatorDirection = Up;
                            result *= -1;
                        } else {
                            elevatorDirection = Down;
                        }
                        requLiftPos = STEPS * result;
                        currLiftPos = 0;
                        
                        if (key < 16)
                        {
                            SetIndicatorElevatorState(requestedElevatorPosition);
                        }
                        else
                        {
                            SetIndicatorFloorState(requestedElevatorPosition);
                        }
                        state = CloseDoor;
                    }
                        
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
                    MoveElevator(elevatorDirection, GetSpeedType());
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

    }

    return (0);
}


/*******************************************************************************
***  PRIVATE FUNCTIONs *********************************************************
*******************************************************************************/

// Add a Button to the circular buffer
uint8_t AddButtonToBuffer(ButtonType button) {
    // Avoid and set write to 0
    if (callBuffer.write >= BUFFER_SIZE) {
        callBuffer.write = 0;
    }
        
    if ( ( callBuffer.write + 1 == callBuffer.read) || ( callBuffer.read == 0 && callBuffer.write + 1 == BUFFER_SIZE ) ) {
        // callBuffer is filled
        return BUFFER_FAIL;
    }
        
    callBuffer.data[callBuffer.write] = button;
    callBuffer.write++;
    if (callBuffer.write >= BUFFER_SIZE){
        // safety first
        callBuffer.write = 0;
    }
        
    return BUFFER_SUCCESS;
}
// Get a Button from the circular buffer
uint8_t GetButtonFromBuffer(ButtonType *button) {
    if (callBuffer.read == callBuffer.write) {
        return BUFFER_FAIL;
    }
        
    *button = callBuffer.data[callBuffer.read];
    callBuffer.read++;
    if (callBuffer.read >= BUFFER_SIZE) {
        callBuffer.read = 0;
    }
        
    return BUFFER_SUCCESS;
}

// Get the speed of the elevator depending on its position
SpeedType GetSpeedType() {
    SpeedType speed = Stop;
    
    if (currLiftPos < 2 || requLiftPos - 2 < currLiftPos) {
        speed = Slow;
    } else if(currLiftPos < 5 || requLiftPos - 5 < currLiftPos) {
        speed = Medium;
    } else {
        speed = Fast;
    }
    
    return speed;
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

