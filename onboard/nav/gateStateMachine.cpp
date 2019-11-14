#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "searchStateMachine.hpp"
#include <cmath>
#include <iostream>

// Constructs a GateStateMachine object with roverStateMachine
GateStateMachine::GateStateMachine( StateMachine* stateMachine_)
    : roverStateMachine( stateMachine_ ) {}

NavState run( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    switch ( phoebe->roverStatus().currentState() )
    {
        case NavState::SearchSpinGate2:
        {
            return executeSearchSpin( phoebe, roverConfig );
        }

        case NavState::SearchSpinWaitGate2:
        case NavState::TurnedToGate2Wait:
        {
            return executeRoverWait( phoebe, roverConfig );
        }

        case NavState::SearchTurnGate2:
        {
            return executeSearchTurn( phoebe, roverConfig );
        }

        case NavState::SearchDriveGate2:
        {
            return executeSearchDrive( phoebe );
        }

        case NavState::DriveThroughGate:
        {
            return executeDriveThroughGate( phoebe );
        }

        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Executes the logic for a search spin. If at a multiple of
// waitStepSize, the rover will go to SearchSpinWaitGate2. If the rover
// detects the Gate 2, it proceeds through the gate. If finished with a 360,
// the rover moves on to the next phase of the search. Else continues
// to search spin.
NavState GateStateMachine::executeSearchSpin( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = roverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if( phoebe->roverStatus().target2In().distance != -1 )
    {
        //check to see if it is the first gate
        if ( phoebe->roverStatus().targetIn().tagBearing != 
                phoebe->roverStatus().target2In().tagBearing )
        {
            //sets the bearing of the second gate and goes to drive through gate
            updateGate2DetectionElements( phoebe->roverStatus().target2In().bearing );
            return NavState::DriveThroughGate;
        }
    }
    if ( nextStop == 0 )
    {
        //get current angle and set as origAngle
        mOriginalSpinAngle = phoebe->roverStatus().odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( phoebe->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::SearchTurnGate2;
        }
        nextStop += waitStepSize;
        return NavState::SearchSpinWaitGate2;
    }
    return NavState::SearchSpinGate2;
} // executeSearchSpin()

// Executes the logic for waiting during a search spin so that CV can
// look for the second gate. If the rover detects gate 2, it proceeds
// through the gate. If the rover is done waiting, it continues the search
// spin. Else the rover keeps waiting.
NavState GateStateMachine::executeRoverWait( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    static bool started = false;
    static time_t startTime;

    if( phoebe->roverStatus().target2In().distance != -1 )
    {
        //check to see if it is the first gate
        if ( phoebe->roverStatus().targetIn().tagBearing != 
                phoebe->roverStatus().target2In().tagBearing )
        {
            //sets the bearing of the second gate and goes to drive through gate
            updateGate2DetectionElements( phoebe->roverStatus().target2In().bearing );
            return NavState::DriveThroughGate;
        }
    }
    if( !started )
    {
        phoebe->stop();
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = roverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        if ( phoebe->roverStatus().currentState() == NavState::SearchSpinWaitGate2 )
        {
            return NavState::SearchSpinGate2;
        }
        return NavState::SearchTurnGate2;
    }
    else
    {
        if ( phoebe->roverStatus().currentState() == NavState::SearchSpinWaitGate2 )
        {
            return NavState::SearchSpinWaitGate2;
        }
        return NavState::TurnedToGate2Wait;
    }
} // executeRoverWait

// Executes the logic for turning while searching.
// If no remaining search points, it proceeds to change search algorithms.
// If the rover detects the second gate, it proceeds through the gate.
// If the rover finishes turning, it proceeds to driving while searching.
// Else the rover keeps turning to the next Waypoint.
NavState GateStateMachine::executeSearchTurn( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    if( mSearchPoints.empty() ) // Do we have another search algorithm to default to? 
    {
        return NavState::ChangeSearchAlg;
    }
    if( phoebe->roverStatus().target2In().distance != -1 )
    {
        //check to see if it is the first gate
        if ( phoebe->roverStatus().targetIn().tagBearing != 
                phoebe->roverStatus().target2In().tagBearing )
        {
            //sets the bearing of the second gate and goes to drive through gate
            updateGate2DetectionElements( phoebe->roverStatus().target2In().bearing );
            return NavState::DriveThroughGate;
        }
    }
    Odometry& nextSearchPoint = mSearchPoints.front();
    if( phoebe->turn( nextSearchPoint ) )
    {
        return NavState::SearchDriveGate2;
    }
    return NavState::SearchTurnGate2;
} // executeSearchTurn()

// Executes the logic for driving while searching.
// If the rover detects the second gate, it proceeds through the gate.
// If the rover finishes driving, it proceeds to turning to the next Waypoint.
// If the rover is still on course, it keeps driving to the next Waypoint.
// Else the rover turns to the next Waypoint or turns back to the current Waypoint
NavState GateStateMachine::executeSearchDrive( Rover* phoebe )
{
    //this needs an update. phoebe->roverStatus() needs another tennis ball object basically
    if( phoebe->roverStatus().target2In().distance != -1 )
    {
        //check to see if it is the first gate
        if ( phoebe->roverStatus().targetIn().tagBearing != 
                phoebe->roverStatus().target2In().tagBearing )
        {
            //sets the bearing of the second gate and goes to drive through gate
            updateGate2DetectionElements( phoebe->roverStatus().target2In().bearing );
            return NavState::DriveThroughGate;
        }
    }
    const Odometry& nextSearchPoint = mSearchPoints.front();
    DriveStatus driveStatus = phoebe->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.pop_front();
        return NavState::SearchSpinGate2;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::SearchDriveGate2;
    }
    return NavState::SearchTurnGate2;
} // executeSearchDrive()


//Executes the traversal through the gate.
NavState GateStateMachine::executeDriveThroughGate( Rover* phoebe )
{
    
}//executeDriveThroughGate

// Sets last known gate 2 angle, so if the second gate is lost, we
// continue to turn to that angle
void GateStateMachine::updateGate2Angle( double bearing )
{
    mGate2Angle = bearing;
} // updateGate2Angle

// Sets last known angles for both the second gate and the rover
// these bearings are used to turn towards the second gate in case the second gate
// is lost while turning
void GateStateMachine::updateGate2DetectionElements( double ball_bearing )
{
    updateGate2Angle( ball_bearing );
} // updateGate2DetectionElements

// The gate factory allows for the creation of gate objects
GateStateMachine* GateFactory ( StateMachine* stateMachine, SearchType type )
{
    GateStateMachine* gate = nullptr;
    type = new LookForGate2Search( stateMachine ); 
    //add more search algorithms???
} //GateFactory

