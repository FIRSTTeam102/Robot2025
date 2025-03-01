// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberOut extends Command {

  private double climberMotorSpeed;
  private Climber climber;
  /** Creates a new ClimberOut. */
  public ClimberOut(Climber climber, double climberMotorSpeed) {
    this.climber = climber;
    this.climberMotorSpeed = climberMotorSpeed;
    
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Climber.getisOut()){
      climber.setClimberPosition(); //runs motor backwards to bring back in if already out
    }
    else{
      climber.setClimberPosition();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((climber.getClimberEncoderPosition()>(Constants.ClimberConstants.climberInPosition) && Climber.getisOut()) || (climber.getClimberEncoderPosition()<Constants.ClimberConstants.climberOutPosition && !Climber.getisOut())){ //if climber is in will end command when position is out or if climber is out will end command when position is in
      climber.toggleisOut(); 
   return false;
    }
    else{
      return true;
    }
  }
}
