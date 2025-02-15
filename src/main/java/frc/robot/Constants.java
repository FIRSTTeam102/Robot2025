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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
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

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class VisionConstants
  {
    public static final boolean DRIVEWITHVISION = true;
  }
//TODO get the CAN id of the sparkmax motor controller & DIO port for the limit switch
  public static final class ElevatorConstants {
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final int ELEVATOR_MOTOR_ID = 30;
    public static final double JStick_Speed_Mult = 0.4;
// TODO: verify values and max height
// The encoder is directly mounted to the elevator shaft - 1 rotation = 1 full rotation of the chain
// sprocket. The diameter of the sprocket is 2", so the circumference = 2 pi.
    public static final double conversionFactor_inches_per_roatation = 2 * Math.PI;
    public static final double motor_max_rpm = 5676;
    public static final double gear_ratio_to_1 = 25;
    public static final double max_linear_ips = ((motor_max_rpm / gear_ratio_to_1) * conversionFactor_inches_per_roatation) / 60;
    public static final double maxHeight_inches = 20;//73;
    
  

    //PID values
    public static double kP = 1; 
    public static double kI = 0.00065;
    public static double kD = 0; 
    public static double kIz = 0.1; 
    public static double kFF = 0.63; 
    public static double kMaxOutput = 0.5; 
    public static double kMinOutput = -0.5;
    public static double maxRPM = 5700;

    /**  Smart Motion Coefficients
    public static double maxVel = 2000; // rpm
    public static double maxAcc = 1500;
	  public static double maxAccel_rpmps = 7200;
	  public static double maxVelocity_rpm = 3800;
    
    // feedforward */
	  public static final double kG = .5;
  }
}
