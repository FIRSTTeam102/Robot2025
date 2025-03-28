// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAuto extends Command {
  Shooter shooter;
  BooleanSupplier coralSensor1Supplier;
  
  /** Creates a new IntakeAuto. */
  public IntakeAuto(Shooter shooter, BooleanSupplier coralSensor1Supplier) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.coralSensor1Supplier = coralSensor1Supplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinShooters(Constants.ShooterConstants.rightIntakeSpeed, Constants.ShooterConstants.leftIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //This has a trigger to stop it since there is no trigger that calls this function, thus there is no trigger that can cause an interrupt.
    //NOTE - sensors are false when they see something & true when they dont :/

    //Only if coral sensor 1 is true - meaning it sees no coral & the shooter says it
    // actually has a coral- stop the motors
    // this is because there is no trigger to start the motors & human player will take time
    // to drop the coral & we don't know how long.
    //Start motors on init - coralSensor1 == true && hasCoral == false -> return false
    //While loading - but hasn't passed coralSensor1: coralSensor1 == false && hasCoral may
    // may not be true -> return false
    //Done loading - coral has passed coralSensor1, so: coralSensor1 == true && hasCoral == true
    // -> return true
    if(coralSensor1Supplier.getAsBoolean() && shooter.hasCoral()){
      return true;
    }
    else{
      return false;
    }
  }
}
