// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants.TargetSide;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  //Create the swerve subsystem using the YAGSL config files
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/Robot2025"));
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds input stream that is 
   * controlled by angular velocity. Invert Joysticks & scale the joystick input (this slows the robot
   * down - so switch this to 0.9 or 1 to request faster speeds)
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)

                                                            .withControllerRotationAxis(()->driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
                                                          

  // create an input stream for simulation - check keyboard translation window in
  //the simulator to adjust keyboard to joystick bindings
 
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  SwerveInputStream driveRobotAngularVelocitySim =  driveAngularVelocitySim.copy().robotRelative(true)
                                                                .allianceRelativeControl(false);                        
  //
  //set up the field & robot oriented drive commands using the input streams, so we are able to get the
  //current joystick values (or keyboard values in simulation)
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveRobotOrientAngularVelocity = drivebase.driveRobotOriented(driveRobotOriented); 
  Command driveRobotOrientAngularVelocitySim = drivebase.driveRobotOriented(driveRobotAngularVelocitySim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("skibidi", driveFieldOrientedAnglularVelocity);

  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                               driveFieldOrientedAnglularVelocity :
                               driveFieldOrientedAnglularVelocitySim);
    
    //switch to Robot oriented driving while right trigger is held in both simulation & live robot
    if (Robot.isSimulation())
    {
      driverXbox.rightTrigger().whileTrue(driveRobotOrientAngularVelocitySim);
    } else {
      driverXbox.rightTrigger().whileTrue(driveRobotOrientAngularVelocity);
    }


    //define the button to command bindings to run in test mode
    if (DriverStation.isTest())
    {
      //drivebase.setDefaultCommand(driveRobotOrientAngularVelocity);
      driverXbox.a().whileTrue(drivebase.sysIdAngleMotorCommand());
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
 //     driverXbox.y().whileTrue(drivebase.driveToPose(
 //           drivebase.getScorePose(TargetSide.LEFT, 6)));
      driverXbox.y().whileTrue(drivebase.driveToPose(drivebase.getScorePose(TargetSide.LEFT, 17)));       
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      
    } else //not in test mode
    { 
      //Field vs Robot oriented drive is defined above - sumlator differences, but Test & other modes are the same

      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
