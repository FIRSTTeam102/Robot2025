// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//TODO test this on the robot

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
  private DigitalInput frontCoralSensor = new DigitalInput(Constants.ShooterConstants.frontShooterSensor);
  private DigitalInput backCoralSensor = new DigitalInput(Constants.ShooterConstants.backShooterSensor);

  public boolean hasCoral = true;


@AutoLogOutput
  private boolean frontCoralSensorOutput; 
  private boolean backCoralSensorOutput;
  public Shooter() {}

  public void spinShooters(double rightMotorSpeed, double leftMotorSpeed){
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
  }
  public boolean getFrontCoralSensor(){
    return frontCoralSensorOutput;
  }

  public boolean getBackCoralSensor(){
    return frontCoralSensorOutput;
  }

  public void stopShooter(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }


  @Override
  public void periodic() {
    if(!(getBackCoralSensor() && getFrontCoralSensor())){
      hasCoral = false;
    }

    if(hasCoral == false){
     if(!getBackCoralSensor()){
         stopShooter();
      }
      else{
        spinShooters(0.25, 0.25);
      }
     if(getFrontCoralSensor()&& !getBackCoralSensor()) {
      hasCoral = true;
     }
    }
  }
}