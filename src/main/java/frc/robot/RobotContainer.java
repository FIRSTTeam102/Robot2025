
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.DrivebaseConstants.TargetSide;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberOutSetSpeed;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualElevatorControl;
import frc.robot.commands.ShootCoral;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
  final         CommandXboxController operatorXbox = new CommandXboxController(1);

  final         CommandXboxController testerXbox = new CommandXboxController(5);

  DigitalInput coralSensor1 = new DigitalInput(Constants.ShooterConstants.coralSensor1Back);
  Trigger funnelTrigger;
  
  // The robot's subsystems and commands are defined here...

  public final Shooter shooter = new Shooter();

  //Create the swerve subsystem using the YAGSL config files
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/Robot2025"));

  private final Elevator elevator = new Elevator();

                                                                        
  private final Climber climber = new Climber();
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
                                                            .scaleTranslation(1.0)
                                                            .scaleRotation(0.5)
                                                            .allianceRelativeControl(true)
                                                            .robotRelative(false);


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
                                                               .robotRelative(false)
                                                               .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(DrivebaseConstants.DriveFastScale)
                                                               .allianceRelativeControl(true);
  SwerveInputStream driveRobotAngularVelocitySim =  driveAngularVelocitySim.copy().robotRelative(true)
                                                                .allianceRelativeControl(false);                        
  //
  //set up the field & robot oriented drive commands using the input streams, so we are able to get the
  //current joystick values (or keyboard values in simulation)
  //Robot oriented default commands are used in TEST mode as the defaults - so no need to hold trigger
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
    NamedCommands.registerCommand("Intake", new Intake(shooter));
    NamedCommands.registerCommand("Shoot", new ShootCoral(shooter, Constants.ShooterConstants.LeftMaxShooterSpeed, Constants.ShooterConstants.RightMaxShooterSpeed));
    NamedCommands.registerCommand("ShootTrough", new ShootCoral(shooter, Constants.ShooterConstants.L1LeftShooterSpeed,Constants.ShooterConstants.L1RightShooterSpeed)); //change the parameters for L1
    NamedCommands.registerCommand("Go to L1",elevator.setElevatorHeight(ElevatorConstants.LEVEL1));
    NamedCommands.registerCommand("Go to L2",elevator.setElevatorHeight(ElevatorConstants.LEVEL2)); //change parameters for L2
    NamedCommands.registerCommand("Go to L3",elevator.setElevatorHeight(ElevatorConstants.LEVEL3)); //change parameters for L3
    NamedCommands.registerCommand("Go to L4",(elevator.setElevatorHeight(ElevatorConstants.LEVEL4))); //change parameters for L4
    NamedCommands.registerCommand("Go to home",(elevator.setElevatorHeight(ElevatorConstants.HOME))); //change parameters for L4

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

    funnelTrigger = new Trigger(coralSensor1::get); //make the trigger and bind it to the funnel sensor
      
    funnelTrigger.whileFalse(new Intake(shooter));

   //default drive mode is field oriented w/ left joystick controlling speed & direction
   //right joystick controls top rotation w/ angular velocity (X axis of right joy stick)
    drivebase.setDefaultCommand(RobotBase.isSimulation() ?
                  drivebase.driveFieldOriented(driveAngularVelocitySim  ) :
                  drivebase.driveFieldOriented(driveAngularVelocity ));
                                
    //Field vs Robot oriented drive is defined above - sumlator differences, but Test & other modes are the same

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().whileTrue(drivebase.centerModulesCommand());
  
  //to change attributes of the default command drive inputstream, just change the 
  //stream attributes that are feeding into the default drive command. The attribute immediately
  //changes and is in effect until it is changed back

  //Drive in precise mode when left trigger is pressed
   driverXbox.leftTrigger().onTrue(Commands.runOnce(
               ()->driveAngularVelocity.scaleTranslation(Constants.DrivebaseConstants.DrivePrecisionScale)
                                       .scaleRotation(0.3)
                                       ))
                           .onFalse(Commands.runOnce(
                ()->driveAngularVelocity.scaleTranslation(Constants.DrivebaseConstants.DriveFastScale)
                                        .scaleRotation(0.5)));
                                        
  //Enable robotRelative driving if the right trigger is pressed.
    driverXbox.rightTrigger().onTrue(Commands.runOnce(
                ()->driveAngularVelocity.robotRelative(true)
                                        .allianceRelativeControl(false)
                                        ))
                              .onFalse(Commands.runOnce(
                ()->driveAngularVelocity.robotRelative(false)
                                        .allianceRelativeControl(true)
                              ));

    driverXbox.leftBumper().whileTrue(drivebase.alignToReefScore(TargetSide.LEFT));
    driverXbox.rightBumper().whileTrue(drivebase.alignToReefScore(TargetSide.RIGHT));
    


    //TODO ???? right bumper used - driverXbox.rightBumper().whileTrue(...) change center of rotation to left or right front corner
    // depending on if left joystick x is left or right

    //TODO driverXbox.a().onTrue(....) toggle robot between field & robot oriented, show on
    // shuffleboard
    driverXbox.a().whileTrue(drivebase.alignToReefScore(17,TargetSide.LEFT));
    driverXbox.b().whileTrue(drivebase.alignToReefScore(18,TargetSide.RIGHT));
      
    // Levels L1, L2, L3, L4 in inches & set to a,b,x,y buttons per Drive team
    //definitions
    operatorXbox.leftBumper().onTrue(elevator.setElevatorHeight(ElevatorConstants.HOME));
    operatorXbox.a().onTrue(elevator.setElevatorHeight(ElevatorConstants.LEVEL1));
    operatorXbox.b().onTrue(elevator.setElevatorHeight(ElevatorConstants.LEVEL2));
    operatorXbox.x().onTrue(elevator.setElevatorHeight(ElevatorConstants.LEVEL3));
    operatorXbox.y().onTrue(elevator.setElevatorHeight(ElevatorConstants.LEVEL4)); 
    operatorXbox.povRight().whileTrue(new ClimberOutSetSpeed(climber,1));
    operatorXbox.povLeft().whileTrue(new ClimberOutSetSpeed(climber, -1));
    
    operatorXbox.rightTrigger().whileTrue(new ShootCoral(shooter, 
                                Constants.ShooterConstants.LeftMaxShooterSpeed,
                                Constants.ShooterConstants.RightMaxShooterSpeed));
    operatorXbox.rightBumper().whileTrue(
         new ShootCoral(shooter,Constants.ShooterConstants.L1LeftShooterSpeed,
                                Constants.ShooterConstants.L1RightShooterSpeed ));

                                
    //TODO flick algae - flip funnel up andThen back
    //TODO flip funnel - up for climb
    //TODO extend climb arm out
    //TODO retract climb arm in to start position
    

    //switch to Robot oriented driving while right trigger is held in both simulation & live robot
    if (Robot.isSimulation())
    {
      driverXbox.rightTrigger().whileTrue(driveRobotOrientAngularVelocitySim);
    } 

    //Left operator joystick controls manual elevator control regardless of mode
		elevator.setDefaultCommand(new ManualElevatorControl(elevator,  () -> operatorXbox.getLeftY() * -1));

    //define the button to command bindings to run in test mode we don't want to run these
    //by accident so we are putting them on a separate xbox controller. NOTE: sysid needs to
    //be run in robot oriented mode
    if (DriverStation.isTest())
    {




      drivebase.setDefaultCommand(driveRobotOrientAngularVelocity);
      testerXbox.a().whileTrue(drivebase.sysIdAngleMotorCommand());
      testerXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
        
      testerXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      testerXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      testerXbox.back().whileTrue(drivebase.centerModulesCommand());
      testerXbox.leftBumper().onTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      //testerXbox.povLeft().onTrue(new FunnelOut(climber, 0.2)); //flip funnel in or out
      //testerXbox.povRight().onTrue(new ClimberOut(climber, 1)); //flip climber in or out
  
    }
    

    }

  
  public void ZeroGyro(){
    drivebase.zeroGyroWithAlliance();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();//TODO this line may be what is breaking the whole thing
    //return drivebase.getAutonomousCommand("New Auto");
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