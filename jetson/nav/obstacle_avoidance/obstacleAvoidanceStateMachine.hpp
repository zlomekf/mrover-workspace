#ifndef OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
#define OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP

#include "rover.hpp"

class StateMachine;

// This class is the representation of different 
// obstacle avoidance algorithms
enum class ObstacleAvoidanceAlgorithm
{
    SimpleAvoidance
};

// This class is the base class for the logic of the obstacle avoidance state machine
class ObstacleAvoidanceStateMachine 
{
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    ObstacleAvoidanceStateMachine( StateMachine* stateMachine_ , Rover* Phoebe, const rapidjson::Document& RoverConfig );

    virtual ~ObstacleAvoidanceStateMachine() {}

    void updateObstacleAngle( double bearing );

    void updateObstacleDistance( double distance );

    void updateObstacleElements( double bearing, double distance );  

    NavState run();

    bool isTargetDetected();

    virtual Odometry createAvoidancePoint( const double distance ) = 0;

    virtual NavState executeTurnAroundObs() = 0;

    virtual NavState executeDriveAroundObs() = 0;

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    
    // Pointer to rover State Machine to access member functions
    StateMachine* roverStateMachine;

    // Odometry point used when avoiding obstacles.
    Odometry mObstacleAvoidancePoint;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleAngle;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleDistance;

    // bool for consecutive obstacle detections
    bool mJustDetectedObstacle;

    // Last obstacle angle for consecutive angles
    double mLastObstacleAngle;

    // Reference to config variables
    const rapidjson::Document& roverConfig;

    // Pointer to rover object
    Rover* phoebe;


};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle 
// avoidance algorithm. This allows for an an ease of transition between obstacle 
// avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory( StateMachine* roverStateMachine,
                                                       ObstacleAvoidanceAlgorithm algorithm );

#endif //OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
