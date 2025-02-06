// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax leftMotor = new SparkMax(Constants.ShooterConstants.leftMotorID,MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Constants.ShooterConstants.rightMotorID, MotorType.kBrushless);

  @AutoLogOutput
  private double leftMotorSpeedOuput= leftMotor.getAppliedOutput();
  @AutoLogOutput
  private double rightMotorSpeedOutput = rightMotor.getAppliedOutput();

  public Shooter() {}

  public void spinShooters(double rightMotorSpeed, double leftMotorSpeed){
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
