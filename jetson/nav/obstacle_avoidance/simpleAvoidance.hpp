#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    SimpleAvoidance( StateMachine* roverStateMachine , Rover* Phoebe, const rapidjson::Document& RoverConfig );

    ~SimpleAvoidance();

    NavState executeTurnAroundObs();

    NavState executeDriveAroundObs();

    Odometry createAvoidancePoint( const double distance );
};

#endif //SIMPLE_AVOIDANCE_HPP
