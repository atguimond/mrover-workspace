#ifndef GATE_STATE_MACHINE_HPP
#define GATE_STATE_MACHINE_HPP

#include "rover.hpp"
#include "./search/searchStateMachine.hpp"

class StateMachine;

enum class Gate2SearchType
{
    LOOKFORGATE2
};

class GateStateMachine
{
    public:
        /*************************************************************************/
        /* Public Member Functions */
        /*************************************************************************/
        GateStateMachine( StateMachine* stateMachine_ );

        virtual ~GateStateMachine() {}

        NavState run( Rover* phoebe, const rapidjson::Document& roverConfig );

    private:
        /*************************************************************************/
        /* Private Member Functions */
        /*************************************************************************/
        NavState executeSearchSpin( Rover* phoebe, const rapidjson::Document& roverConfig );

        NavState executeRoverWait( Rover* phoebe, const rapidjson::Document& roverConfig );

        NavState executeSearchTurn( Rover* phoebe, const rapidjson::Document& roverConfig );

        NavState executeSearchDrive( Rover* phoebe );

        NavState executeDriveThroughGate( Rover* phoebe );

        void updateGate2DetectionElements( double ball_bearing );

        void updateGate2Angle( double bearing );

        /*************************************************************************/
        /* Private Member Variables */
        /*************************************************************************/

        // Last known angle to turn to tennis ball.
        double mGate2Angle;

        // Last known angle of rover from turn to tennis ball.
        double mTurnToGate2RoverAngle;

    protected:
        /*************************************************************************/
        /* Protected Member Variables */
        /*************************************************************************/

        // Pointer to rover State Machine to access member functions
        StateMachine* roverStateMachine;

        // Vector of search point multipliers used as a base for the search points.
        vector< pair<short, short> > mSearchPointMultipliers;

        // Queue of search points.
        deque<Odometry> mSearchPoints;
};

// Creates an GateStateMachine object
GateStateMachine* GateFactory( StateMachine* stateMachine, SearchType type );

#endif //GATE_STATE_MACHINE_HPP

