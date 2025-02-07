// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax leftMotor = new SparkMax(Constants.ShooterConstants.leftMotorID,MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Constants.ShooterConstants.rightMotorID, MotorType.kBrushless);
  private DigitalInput coralSensor = new DigitalInput(Constants.ShooterConstants.shooterSensor);

  @AutoLogOutput
  private double leftMotorSpeedOuput= leftMotor.getAppliedOutput();
  @AutoLogOutput
  private double rightMotorSpeedOutput = rightMotor.getAppliedOutput();
@AutoLogOutput
  private boolean coralSensorOutput; 
  public Shooter() {}

  public void spinShooters(double rightMotorSpeed, double leftMotorSpeed){
    // TODO:  Make sure its not voltage pls
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
  }

  public boolean getCoralSensor(){
    return coralSensorOutput;
  }



  @Override
  public void periodic() {
  coralSensorOutput = coralSensor.get();  }
}
