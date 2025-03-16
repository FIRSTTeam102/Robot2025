// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148.0 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    public static double DriveFastScale = 1
    ;
    public static double DrivePrecisionScale = 0.35;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10.0; // seconds
    public enum TargetSide {LEFT, RIGHT};
    // robot camera offsets need to be correct with bumper so the 
    //align to reef works correctly, the reef poles are 6.5 inches from the 
    //center of the april tag
    public static double ReefLeftYOffset = Units.inchesToMeters(-7.5);
    public static double ReefRightYOffset = Units.inchesToMeters(7.5);
    public static double ReefXDistance = Units.inchesToMeters(14.0);                                                                                             
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT    = 6.0;
    
  }
  public static class VisionConstants
  {
    public static final boolean DRIVEWITHVISION = true;
    public static final double oldCameraX = 10.6488; // Old val  11.97534
    public static final double oldCameraY = 11.957134; // old val 11.256

  }

  //TODO get the DIO port for the limit switch
  public static final class ElevatorConstants {

    public static final double NATIVE_UNITS_PER_ROTATION = 8192.0;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final int ELEVATOR_MOTOR_ID = 30;
    public static final double JStick_Speed_Mult = 0.6;
    public static final double OUTPUT_VOLTS = 1.0;
    public static final double HOME = 0.00
    ;
    public static final double LEVEL1 = 26.479;
    public static final double LEVEL2 = 37.6415;
    public static final double LEVEL3 = 56.0746; 
    public static final double LEVEL4 = 80.7107;


// The encoder is directly mounted to the elevator shaft - 1 rotation = 1 full rotation of the chain
// sprocket. The diameter of the sprocket is 2", so the circumference = 2 pi. This is a 3 stage elevator &
// the shooter is attached to the 3rd stage, so it moves at 3x the rate of the chain distance
    public static final double inches_per_rotation = 2.0 * Math.PI * 3.0;
    public static final double maxHeight_inches = 80.0; 
    public static final double carriageMass = 23.0; //lbs shooter + elevator carriage + chain 
    //multiplied maxHeight Rotations by native units to get max native units
    public static final double maxHeight_rotations = (maxHeight_inches/inches_per_rotation);
    public static final double rotations_per_inch = 1.0/inches_per_rotation;
    public static final double secondsToMaxHeight = 2.0; //top speed goal
    public static final double maxRotationsPerMin = (maxHeight_rotations/secondsToMaxHeight) * 60;
    public static final double gear_ratio_to_1 = 25.0; //NEO gear ratio
    public static final double motor_max_rpm = 1000.0;//maxRotationsPerMin * gear_ratio_to_1; //Goal of max height in 2 seconds
    public static final double motor_max_accel = 3000.0;//TODO tune this to achieve fast ramp up

    public static final double ElevatorDefaultToleranceInch = 0.2; //1 inch either way error in position
    public static final double ElevatorDefToleranceRotations = ElevatorDefaultToleranceInch /inches_per_rotation;
    

    //PID values - determined using Rev hardware client & graphing velocity
    public static double kP = 0.00025; 
    public static double kI = 0.0;
    public static double kD = 0.0; 
    public static double kIz = 0.1; 
    public static double kFF = 0.002114; //velocity feed forward = 1/kv = 1/473 per neo docs 
    public static double kMaxOutput = 1.0; 
    public static double kMinOutput = -1.0;
    
    // feedforward */
	  public static final double kG = 0.1056 * .01957;//effect of elevator gravity on neo 
    public static final double KV = .5; //for a neo
  }
  public static class ShooterConstants{
    //CAN ID's
    public static final int RightMotorID = 33; 
    public static final int LeftMotorID = 32;

    public static final int coralSensor2Front = 2; 
    public static final int coralSensor1Back = 1;

    //shooter speeds
    public static final double LeftMaxShooterSpeed = -0.2;
    public static final double RightMaxShooterSpeed = 0.2;


    public static final double L1LeftShooterSpeed = -0.3;
    public static final double L1RightShooterSpeed = -0.1;

    public static final double leftIntakeSpeed = 0.05;
    public static final double rightIntakeSpeed = -0.05;

  }
  public static class ClimberConstants {
    public static final int climberMotorID = 31; 
    public static final int funnelMotorID = 40; //temporary


    //positions for the climber
    public static final int climberInPosition = 0; //might need testing to find correct position or maybe it is just 0
    public static final int climberOutPosition = 60; //temporary needs testing to find correct position


    //positions for the funnel
    public static final double funnelOutPosition = 0.9; //temporary
    public static final double funnelInPosition = 0.9; //temporary

    public static final double climberKp =1;
    public static final double climberKi =0;
    public static final double climberKd =0;

    public static final double funnelKp =0.15;
    public static final double funnelKi =0;
    public static final double funnelKd =0.05;


    public static final int gearboxConversion = 125;

    //range outputs for PID
    public static final int maxClimberRotatations =1 ;  //temporary, number should be rotations of the GEARBOX.
    public static final double maxFunnelRotatations =0.5 ;  //temporary, probably just 1 or less since the funnel is a 1:1 ratio

  }
}