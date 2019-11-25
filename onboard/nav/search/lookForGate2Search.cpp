#include "lookForGate2Search.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>
#include "./rover_msgs/Odometry.hpp"

LookForGate2Search::~LookForGate2Search() {}

// Clears the search points
void LookForGate2Search::initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    mSearchPoints.clear();

    //change to while we want to keep looking for the second post. Hope we do not fuck up.
    
    
    //insertIntermediatePoints( phoebe, roverConfig );
} // initializeSearch()