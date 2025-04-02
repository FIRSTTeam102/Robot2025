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
  BooleanSupplier isIntakingSupplier;
 // BooleanSupplier coralSensor1Supplier;
  
  /** Creates a new IntakeAuto. */
  public IntakeAuto(Shooter shooter, BooleanSupplier isIntakinBooleanSupplier) {  //BooleanSupplier coralSensor1Supplier
    this.shooter = shooter;
    this.isIntakingSupplier = isIntakinBooleanSupplier;
    addRequirements(shooter);
    
    //this.coralSensor1Supplier = coralSensor1Supplier;
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
  public boolean isFinished() { 
    
    // intaking is done when the Intaking Sensor (coral sensor1 does not see a coral) &&
    //the shooter has a coral
    if (!isIntakingSupplier.getAsBoolean() && shooter.hasCoral()){
      return true;
    }
    else{
      return false;
    }
  }
}
