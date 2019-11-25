#ifndef LOOK_FOR_GATE2_SEARCH_HPP
#define LOOK_FOR_GATE2_SEARCH_HPP

#include "gateStateMachine.hpp"

/*************************************************************************/
/* LookForGate2rr Search */
/*************************************************************************/
class LookForGate2Search : public GateStateMachine
{
public:
    LookForGate2Search( StateMachine* stateMachine_ )
    : GateStateMachine(stateMachine_) {}

    ~LookForGate2Search();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig );
};

#endif //LOOK_FOR_GATE2_SEARCH_HPP