#include "LaserScanner.h"
#include "VelocityController.h"
#include "TurtleController.h"

#include "ros/ros.h"

#include <math.h>
#include <cmath>    // isnan

/* --------------------------------------------------------------------
CONTROL CONSTANTS
--------------------------------------------------------------------- */

const float FORWARD_VELOCITY =              0.15;
const float ANGULAR_VELOCITY =              1.0;
const float WALL_FOLLOW_DISTANCE =          0.20;
const float WALL_FOLLOW_MARGIN =            0.07;
const float WALL_STOP_DISTANCE =            0.20;
const float WALL_CHECK_DISTANCE =           0.45;

const float MAX_INTEGRAL_ERROR =            3.0;
const float K_PROPORTIONAL_SPIN =           0.7;
const float K_INTEGRAL_SPIN =               0.005;
const float K_PROPORTIONAL_HEADING =        1.5;
const float K_INTEGRAL_HEADING =            0.0;
const float K_DERIVATIVE_HEADING =          0.0;
const float K_PROPORTIONAL_WALLFOLLOW =     1.5;
const float K_INTEGRAL_WALLFOLLOW =         0.0;
const float K_DERIVATIVE_WALLFOLLOW =       0.3;
const float MAX_INTEGRAL_ERROR_WALLFOLLOW = 1.5;

const float U_MAX_ANGULAR_WALLFOLLOW = 0.1;


TurtleController::TurtleController(LaserScanner * laserscan, VelocityController * velocitycontroller){
	this->laserscan = laserscan;
	this->velocitycontroller = velocitycontroller;
	this->robotState = INSIDE_WALL;
}




void TurtleController::run(){
    LaserStats north = this->laserscan->getLaserStats(350, 10);
    LaserStats west = this->laserscan->getLaserStats(80, 110);
    LaserStats east = this->laserscan->getLaserStats(260, 280);
    LaserStats south = this->laserscan->getLaserStats(170, 190);

	std::cout << "--- Distance Ahead: " << north.min << std::endl;
	std::cout << "--- Distance Left:  " << west.min << std::endl;
	std::cout << "--- Distance Right: " << east.min << std::endl;
	std::cout << "--- Distance Behind: " << south.min << std::endl;

    if (this->robotState == ACTION_COMPLETE){
        std::cout << "[RobotState]: Action completed... Deciding next state" << std::endl;
        this->robotState = this->decideNextAction();
    }

    if (this->robotState == FOLLOWING_WALL){
        std::cout << "[RobotState]: Following Wall" << std::endl;
        this->followWall();
    }

    if (this->robotState == OUTSIDE_WALL){
        std::cout << "[RobotState]: Sweeping around outside wall with r " << this->sweepRadius << std::endl;
        this->followOutsideWall();
    }

    if (this->robotState == INSIDE_WALL){
        std::cout << "[RobotState]: Escaping inside corner " << std::endl;
        this->followInsideWall();
    }
}



void TurtleController::followInsideWall(){
	if(this->checkInsideWallExitCondition()){
		this->velocitycontroller->move({0,0,0},{0,0,0});
		this->robotState = ACTION_COMPLETE;
		return;
	}

	this->velocitycontroller->move({0,0,0},{0,0,-1*ANGULAR_VELOCITY});
}




void TurtleController::followOutsideWall(){
    if (this->checkOutsideWallExitCondition()){
        this->velocitycontroller->move({0,0,0}, {0,0,0});
        this->robotState = ACTION_COMPLETE;
        return;
    }

    float u_angular = FORWARD_VELOCITY / this->sweepRadius;
	this->velocitycontroller->move({FORWARD_VELOCITY, 0, 0}, {0, 0, u_angular});
}





void TurtleController::followWall(){
    
    if (this->checkWallFollowExitCondition()){
        this->velocitycontroller->move({0,0,0},{0,0,0});
        this->robotState = ACTION_COMPLETE;
        return;
    }

    LaserStats laserNorth = this->laserscan->getLaserStats(340, 10);
	LaserStats laserWest = this->laserscan->getLaserStats(80, 100);

	float errorLeft = laserWest.min - WALL_FOLLOW_DISTANCE;
	float derivativeErrorLeft;
	static float integralErrorLeft = 0;
	static float previousError = 0;

    float u_angular;
    float u_linear = FORWARD_VELOCITY;

    // If the error is pretty small, stay aligned with the wall
	if (fabs(errorLeft) < WALL_FOLLOW_MARGIN){
		float heading = this->laserscan->measureHeadingFromLeftWall();
		float heading_error = -1*heading;
		u_angular = K_PROPORTIONAL_HEADING * heading_error;	

        // If the magnitude of the heading error is too large, stop and correct.
		if (fabs(heading_error) > 0.2){ 
            u_linear = 0; 
        }
	}
    else{
		integralErrorLeft += errorLeft;
		derivativeErrorLeft = errorLeft - previousError;
		
		// Saturate integral error so we don't get integral ramping
        saturateSignal(integralErrorLeft, MAX_INTEGRAL_ERROR_WALLFOLLOW);

		u_angular = K_PROPORTIONAL_WALLFOLLOW * errorLeft + 
                    K_INTEGRAL_WALLFOLLOW * integralErrorLeft + 
                    K_DERIVATIVE_WALLFOLLOW * derivativeErrorLeft;
		
        // Saturate the angular control signal at U_MAX_ANGULAR_WALLFOLLOW
        saturateSignal(u_angular, U_MAX_ANGULAR_WALLFOLLOW);        
	}
    previousError = errorLeft;
    nanToZero(u_angular);	
    this->velocitycontroller->move({u_linear, 0, 0}, {0, 0, u_angular});
    return;
}




bool TurtleController::checkWallFollowExitCondition(){
    LaserStats laserNorthWest = this->laserscan->getLaserStats(45, 55);
    LaserStats laserWest = this->laserscan->getLaserStats(45, 135);
    LaserStats laserNorth = this->laserscan->getLaserStats(340, 20);

    if (laserNorthWest.min > WALL_CHECK_DISTANCE * 1.2){
        std::cout << "[WallFollowExitCondition]: Ran out of wall to follow" << std::endl;
        return true;
    }    
    if (laserWest.min >= WALL_CHECK_DISTANCE){
        std::cout << "[WallFollowExitCondition]: The wall on the left just disappeared?" << std::endl;
        return true;
    }
    if (laserNorth.min <= WALL_STOP_DISTANCE){
        std::cout << "[WallFollowExitCondition]: There is a wall ahead" << std::endl;
        return true;
    }
    return false;
}




bool TurtleController::checkWallFollowEntryCondition(){
    LaserStats laserWest = this->laserscan->getLaserStats(85, 95);
    LaserStats laserNorth = this->laserscan->getLaserStats(350, 10);
    LaserStats laserNorthWest = this->laserscan->getLaserStats(45, 55);

    // No wall ahead and a wall immediately to the left
    if (laserNorth.min >= WALL_STOP_DISTANCE && laserWest.min <= WALL_FOLLOW_DISTANCE * 1.5){
        return true;
    }
    return false;
}




bool TurtleController::checkOutsideWallExitCondition(){
    LaserStats laserNorthWest = this->laserscan->getLaserStats(45, 55);
    LaserStats laserNorth = this->laserscan->getLaserStats(350, 10);

    if (laserNorthWest.median < WALL_CHECK_DISTANCE){
        return true;
    }
    if (laserNorth.min < WALL_STOP_DISTANCE){
        return true;
    }
    return false;
}





bool TurtleController::checkOutsideWallEntryCondition(){
    LaserStats laserWest = this->laserscan->getLaserStats(80, 110);
    LaserStats laserNorthWest = this->laserscan->getLaserStats(45, 55);

    // No wall ahead to the left, and a wall immediately to the left
    if (laserNorthWest.min > WALL_CHECK_DISTANCE * 1.2 && laserWest.min <= WALL_CHECK_DISTANCE){
           return true;
    }
    return false;
}




bool TurtleController::checkInsideWallEntryCondition(){
    LaserStats laserWest = this->laserscan->getLaserStats(80, 100);
    LaserStats laserNorth = this->laserscan->getLaserStats(350, 10);

    if (laserWest.min < WALL_CHECK_DISTANCE && laserNorth.min <= WALL_STOP_DISTANCE * 1.2){
        return true;
    }
    return false;
}




bool TurtleController::checkInsideWallExitCondition(){
    LaserStats laserNorth = this->laserscan->getLaserStats(350, 10);

    if (laserNorth.min > WALL_STOP_DISTANCE+0.1){
        return true;
    }
    return false;
}




float saturateSignal(float & signal, float saturationLevel){
    if (signal > saturationLevel){
        signal = saturationLevel;
    }
    else if(signal < -1 * saturationLevel){
        signal = -1 * saturationLevel;
    }
    return signal;
}




void nanToZero(float &signal){
    if (std::isnan(signal)){
        signal = 0;
    }
}




TurtleControllerState TurtleController::decideNextAction(){
    if (this->checkOutsideWallEntryCondition()){
        LaserStats laserWest = this->laserscan->getLaserStats(80, 110);
		this->sweepRadius = laserWest.min * 1.5;
        saturateSignal(this->sweepRadius, 0.4);
		return OUTSIDE_WALL;
	}

    if (this->checkWallFollowEntryCondition()){
        return FOLLOWING_WALL;
    }

    if (this->checkInsideWallEntryCondition()){
        return INSIDE_WALL;
    }
    return DO_NOTHING;
}


