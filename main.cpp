/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       macos                                                     */
/*    Created:      2/7/2025, 10:38:08 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/




#include "vex.h"

#include <algorithm>

using namespace vex;

vex::task liftTask;



// A global instance of competition
competition Competition;

controller Controller1;

brain Brain;



// Define three motors for the left side
motor leftMotor1(PORT1, ratio6_1, true);
motor leftMotor2(PORT2, ratio6_1, true);
motor leftMotor3(PORT3, ratio6_1, true);


// Define three motors for the right side
motor rightMotor1(PORT4, ratio6_1, false);
motor rightMotor2(PORT5, ratio6_1, false);
motor rightMotor3(PORT9, ratio6_1, false);


motor liftMotor(PORT6, ratio6_1, false);


// Intake Motor
motor intakeMotor2(PORT7, ratio6_1, true); // Ramp




// Clamp
digital_out clamp(Brain.ThreeWirePort.A);


// Doinker
digital_out doinker(Brain.ThreeWirePort.B);



// Group left and right motors for drivetrain
motor_group leftDrive(leftMotor1, leftMotor2, leftMotor3);
motor_group rightDrive(rightMotor1, rightMotor2, rightMotor3);


// Group intake motors
motor_group intake(intakeMotor2);


// Intertial Sensor


inertial InertialSensor(PORT8);


// Define drivetrain with additional parameters
// Wheel travel (circumference): 3.25 inches diameter * π
// Track width: distance between left and right wheels (e.g., 12 inches)
// Wheelbase (optional): distance between front and back wheels (adjust as needed, e.g., 10 inches)
// Distance units: inches
drivetrain Drivetrain(leftDrive, rightDrive, 3.25 * M_PI, 12.0, 10.0, distanceUnits::in);



// Function for moving forward a specified distance in inches
void moveForward(double distance, double speed) {
Drivetrain.driveFor(distance, distanceUnits::in, speed, velocityUnits::pct);
}



// Function for moving backward a specified distance in inches
void moveBackward(double distance, int speed) {
Drivetrain.driveFor(directionType::rev, distance, distanceUnits::in, speed, velocityUnits::pct);
}


// Function for turning right a specified number of degrees
void turnRight(double degrees, int speed) {
Drivetrain.turnFor(turnType::right, degrees, rotationUnits::deg, speed, velocityUnits::pct);
}



// Function for turning left a specified number of degrees
void turnLeft(double degrees, int speed) {
Drivetrain.turnFor(turnType::left, degrees, rotationUnits::deg, speed, velocityUnits::pct);
}


void activateIntake(double degrees, int percent){
intake.spinFor(directionType::fwd, degrees, rotationUnits::deg, percent, velocityUnits::pct); // Activate intake motor group
}


void activateOuttake(double degrees, int percent){
intake.spinFor(directionType::rev, degrees, rotationUnits::deg, percent, velocityUnits::pct); // Activate outtake motor group
}



// Activating the clamp
void activateClamp() {
    clamp.set(true); // Engage the clamp
}

// Deactivating the clamp
void deactivateClamp() {
    clamp.set(false); // Release the clamp
}



void pre_auton() {
    Brain.Screen.print("Pre-autonomous setup complete.");
    
    // Make sure the pneumatic piston is retracted when robot is powered on
    clamp.set(false); // Set to false to retract piston
    doinker.set(false);
}


// Custom clamp function to limit values between min and max
double limitValue(double value, double min, double max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}


// Constants for PID control
double liftkP = 0.275;  // Tune this for performance
double liftkI = 0.00025;
double liftkD = 0.015;
int liftTarget = 0; // Global variable for target lift position
bool liftTaskRunning = false; // Track if the lift PID task is active

int liftUpPosition = -950;   // Position when L1 is pressed (moves up)
int liftDownPosition = 100; // Position when L2 is pressed (move opposite direction)
int liftMidPosition = 0;    // Middle position when Down button is pressed
int liftTopPosition = -130; // Position when top button is pressed

int liftPIDTask() {
    double liftIntegral = 0.0; // Integral term for PID
    int liftPrevError = 0;     // Previous error for derivative term
    int liftPosition;
    int liftError;
    int liftDerivative;

    // Reset encoder and start PID task
    liftMotor.setPosition(0, degrees); // Reset encoder to 0

    while (true) {
        // Get the current lift position
        liftPosition = liftMotor.position(degrees);
        
        // Calculate error
        liftError = liftTarget - liftPosition;
        
        // Derivative term
        liftDerivative = liftError - liftPrevError;

        // Accumulate error (integral term)
        liftIntegral += liftError;

        // PID calculation
        double liftPower = (liftError * liftkP) + (liftDerivative * liftkD) + (liftIntegral * liftkI) * 1.25;

        // Apply power to the lift motor
        liftMotor.spin(directionType::fwd, liftPower, velocityUnits::pct);

        // Update previous error
        liftPrevError = liftError;

        // Check if close to the target (tolerance)
        if (abs(liftError) < 2) {
            liftMotor.stop(brakeType::hold); // Apply brake hold
            break; // Exit the loop when the target is reached
        }

        vex::task::sleep(20); // Small delay to prevent task overload
    }

    liftTaskRunning = false; // Mark task as stopped
    return 0;
}



// PID Move

double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Autonomous setting


int desiredValue = 200;
int desiredTurnValue = 0;
int error; // SensorValue - DesiredValue : Position
int prevError = 0; // Position 20 milliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; // totalError = totalError + error








int turnError; // SensorValue - DesiredValue : Position
int turnPrevError = 0; // Position 20 milliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; // totalError = totalError + error








bool resetDriveSensors = false;








// Variable modified for use
bool drivePIDOn = true;




int drivePID(){

 while(drivePIDOn){

   if(resetDriveSensors){
     resetDriveSensors = false;


     leftDrive.setPosition(0,degrees);
     rightDrive.setPosition(0,degrees);
   }

   /////////////////////
   // LATERAL MOVEMENT PID
   ////////////////////////////
   int leftMotorPosition = leftDrive.position(degrees);
   int rightMotorPosition = rightDrive.position(degrees);
   int averagePosition = (leftMotorPosition + rightMotorPosition) / 2.0;

   //Proportional
   error = averagePosition - desiredValue;

   //Derivative
   derivative = error - prevError;

   //Velocity -> Position : Integral
   //totalError += error;

   double lateralMotorPower = (error * kP + derivative * kD);


   ////////////////////////////////
   // TURNING MOVEMENT PID
   /////////////////////////////////////////////

   int turnDifference = leftMotorPosition - rightMotorPosition;


   //Proportional
   turnError = averagePosition - desiredValue;


   //Derivative
   turnDerivative = turnError - turnPrevError;

   //Velocity -> Position : Integral
   //turnTotalError += turnError;


   double turnMotorPower = (error * turnkP + derivative * turnkD);




   /////////////////////////////////////////////




   leftDrive.spin(directionType::fwd, lateralMotorPower + turnMotorPower, velocityUnits::pct);
   rightDrive.spin(directionType::fwd, lateralMotorPower - turnMotorPower, velocityUnits::pct);



   //code
   prevError = error;
   turnPrevError = turnError;
   vex::task::sleep(20);


 }


return 1;
}



// Autonomous function
void autonomous() {

  activateClamp();

  moveBackward(13.0, 25);

  vex::task::sleep(250);

  turnRight(78.5, 25);

  moveBackward(14.5, 15);

  moveForward(0.5, 25);

  activateIntake(1420, 100);
  
  


    }





// Driver control function
void usercontrol() {
 
  bool clampEngaged = false; // Boolean variable to track clamp state

  bool doinkerEngaged = true; // Boolean variable to track doinker state

  while (true) {
       // Read joystick values
        int forward = Controller1.Axis3.position() * 0.90; // Left joystick for forward/backward
        int turn = Controller1.Axis1.position() * 0.9 * 0.375;    // Right joystick for turning


         if (abs(forward) < 5) { // If nearly stationary
            turn *= 0.5; // Reduce turn speed by 50%
        }

        
        // Calculate motor speeds for arcade drive
        int leftSpeed = forward + turn;
        int rightSpeed = forward - turn;

        // Apply motor speeds
        leftDrive.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
        rightDrive.spin(directionType::fwd, rightSpeed, velocityUnits::pct);


        // Small delay to prevent CPU overloading
        vex::task::sleep(20);




      // Intake control using R1 and R2
      /*if (Controller1.ButtonR1.pressing()) {
          intakeMotor.spin(directionType::rev, 75, velocityUnits::pct); // Intake forward
          intakeMotor2.spin(directionType::rev, 75, velocityUnits::pct);
      } 
      if (Controller1.ButtonR2.pressing()) {
          intakeMotor.spin(directionType::fwd, 75, velocityUnits::pct); // Intake reverse
          intakeMotor2.spin(directionType::fwd, 75, velocityUnits::pct);
      }
      */
      if (Controller1.ButtonR1.pressing()) {
          
          intakeMotor2.spin(directionType::fwd, 100, velocityUnits::pct);
      } 
     else {if (Controller1.ButtonR2.pressing()) {
          intakeMotor2.spin(directionType::rev, 80, velocityUnits::pct);
      }
       else{
         
         intakeMotor2.stop();
       }
      }
             // Clamp control: Toggle using Button B
      if (Controller1.ButtonB.pressing()) {
          clampEngaged = !clampEngaged;        
          clamp.set(clampEngaged);  // Engage/disengage clamp
          vex::task::sleep(300);             
      }

      if(Controller1.ButtonA.pressing()){
          doinkerEngaged = !doinkerEngaged;
          doinker.set(doinkerEngaged);
          vex::task::sleep(300);
      }

// Variable to store the lift target position
int storedLiftTarget = 0;

  if (Controller1.ButtonL1.pressing() && !liftTaskRunning) {
        liftTarget = liftUpPosition;  
        liftTaskRunning = true;
        liftTask = vex::task(liftPIDTask);
    }

    if (Controller1.ButtonL2.pressing() && !liftTaskRunning) {
        liftTarget = (liftDownPosition * 4.5);  
        liftTaskRunning = true;
        liftTask = vex::task(liftPIDTask);
    }

    if (Controller1.ButtonDown.pressing() && !liftTaskRunning) {
        liftTarget = liftDownPosition;  
        liftTaskRunning = true;
        liftTask = vex::task(liftPIDTask);
    }

    if (Controller1.ButtonUp.pressing() && !liftTaskRunning) {
        liftTarget = liftTopPosition;  
        liftTaskRunning = true;
        liftTask = vex::task(liftPIDTask);
    }




      vex::task::sleep(20); // Short delay for loop stability
  }

}








int main() {
  pre_auton();
  autonomous();
  // Competition.autonomous(autonomous);
  // Competition.drivercontrol(usercontrol);




  while (true) {
      vex::task::sleep(100);
  }
}


