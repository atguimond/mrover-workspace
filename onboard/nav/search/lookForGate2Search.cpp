#include "lookForGate2Search.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>

LookForGate2::~LookForGate2() {}

// Clears the search points
void LookForGate2::initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    mSearchPoints.clear();

    //change to while we want to keep looking for the second post. Hope we do not fuck up.
    while( ) 
    {
        //for 3 because we want three search points??
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            //these need to be changed but I do not know how to change them.
            Odometry nextSearchPoint = phoebe->roverStatus().path().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * visionDistance  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * visionDistance * phoebe->longMeterInMinutes() );
            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            mSearchPoints.push_back( nextSearchPoint );

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;

        }
    }
    insertIntermediatePoints( phoebe, roverConfig );
} // initializeSearch()