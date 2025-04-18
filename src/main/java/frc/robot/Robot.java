// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.AnimationTypes;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  @AutoLogOutput
  private boolean hasCoral = false;

  @AutoLogOutput
  private boolean isDistanceGood = false;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    m_robotContainer.setLightPattern(AnimationTypes.Rainbow);

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();


    /** LIGHTS
     * 1. if elevator is at l4
     *  set lights to l4 color
     * 2. elif elevator is at algae
     *  set lights to algae color
     * 3. elif hascoral = true
     *  set lights to hascoral
     * 4. else
     *  alliance color
     */
   
    hasCoral = m_robotContainer.hasCoral();
    isDistanceGood = m_robotContainer.alignDistanceCheck();

    if(isDisabled()){
      m_robotContainer.setLightPattern(AnimationTypes.Rainbow);
    } 
    else if(m_robotContainer.getElevatorHeight(ElevatorConstants.LEVEL4, 4)){
      m_robotContainer.setLightPattern(AnimationTypes.LEVEL4);
    }
    else if(m_robotContainer.getElevatorHeight(ElevatorConstants.ALGAE1, 2)||
      m_robotContainer.getElevatorHeight(ElevatorConstants.ALGAE2, 2)){
      m_robotContainer.setLightPattern(AnimationTypes.ALGAE);
    }
    else if(m_robotContainer.alignDistanceCheck()){
      m_robotContainer.setLightPattern(AnimationTypes.Stuck);
    }
    else if (m_robotContainer.hasCoral()){
      m_robotContainer.setLightPattern(AnimationTypes.CORAL);
    }
    else{
      m_robotContainer.setDefaultLights();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    m_robotContainer.setLightPattern(AnimationTypes.Rainbow);
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
     {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.ZeroGyro();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.setDefaultLights();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setDefaultLights();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
