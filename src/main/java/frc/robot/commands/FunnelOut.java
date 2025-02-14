// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FunnelOut extends Command {

  private double funnelMotorSpeed;
  private Climber climber;
  /** Creates a new FunnelOut. */
  public FunnelOut(Climber climber, double funnelMotorSpeed) {
    this.climber = climber;
    this.funnelMotorSpeed = funnelMotorSpeed;

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Climber.getfunnelOut()){ //ends command instantly if funnel is already out but probably not the most efficient way to do this
    climber.setFunnelMotorSpeed(funnelMotorSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setFunnelMotorSpeed(0); 
    climber.togglefunnelOut(); //may want to make it possible to put funnel back in incase of accidental press
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.getFunnelMotorCurrent()>=Constants.ClimberConstants.funnelMotorStallCurrentAmps){
      return false;
    }
    else if(Climber.getfunnelOut()){ 
      return false;
    }
    else{
      return true;
    }
  }
}
