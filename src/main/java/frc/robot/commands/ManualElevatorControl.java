// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevatorControl extends Command {
  private Elevator elevator;
  private DoubleSupplier joystick;

  public ManualElevatorControl(Elevator elevator, DoubleSupplier joystick) {

      this.elevator = elevator;
      this.joystick = joystick;
      addRequirements(elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.manualMove(MathUtil.applyDeadband(joystick.getAsDouble(),OperatorConstants.LEFT_Y_DEADBAND)
                                                                     * ElevatorConstants.JStick_Speed_Mult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
