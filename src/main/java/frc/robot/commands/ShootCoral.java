// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {
  /** Creates a new ShootCoral. */

  private double leftMotorSpeed;
  private double rightMotorSpeed;
  private Shooter shooter;

  public ShootCoral(Shooter shooter, double leftMotorSpeed, double rightMotorSpeed) {
    this.shooter = shooter;
    this.leftMotorSpeed = leftMotorSpeed;
    this.rightMotorSpeed = rightMotorSpeed;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinShooters(leftMotorSpeed,rightMotorSpeed);
   System.out.println("shooting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinShooters(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //if the coral sensor sees coral, keep motors running, if the coral is not sensed, stop the motors
     if(!(shooter.getCoralSensor2())){ 
      return false;
    }
    else{
      return true;
    }
  }
}
