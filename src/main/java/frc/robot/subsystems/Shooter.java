// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//TODO test this on the robot

package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private SparkMax RightMotor = new SparkMax(Constants.ShooterConstants.RightMotorID,MotorType.kBrushless);
  private SparkMax LeftMotor = new SparkMax(Constants.ShooterConstants.LeftMotorID, MotorType.kBrushless);
  private DigitalInput coralSensor2 = new DigitalInput(Constants.ShooterConstants.coralSensor2Front);
 // private DigitalInput coralSensor1 = new DigitalInput(Constants.ShooterConstants.coralSensor1Back);
@AutoLogOutput
  public boolean hasCoral = false;

  BooleanPublisher backCoralSensorPublisher;

@AutoLogOutput

  //private boolean coralSensor1Output;
  private boolean coralSensor2Output;

  //private Boolean coralSensor3BooleanSupplier;

  public Shooter() {
    motorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(true);


    RightMotor.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    LeftMotor.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  }

  public void spinShooters(double rightMotorSpeed, double leftMotorSpeed){
    RightMotor.set(rightMotorSpeed);
    LeftMotor.set(leftMotorSpeed);
  }
  //public boolean getCoralSensor1(){
  //  return coralSensor1.get();
 // }
  public boolean getCoralSensor2(){
    return coralSensor2.get();
  }

  public void stopShooter(){
    RightMotor.stopMotor();
    LeftMotor.stopMotor();
  }
/* 
  public void hasCoralToggle(){
    hasCoral=!hasCoral;
  }
*/
  public boolean hasCoral(){
    //return hasCoral;
    return (!coralSensor2.get());
  }
  

  @Override
  public void periodic() {
    hasCoral = !coralSensor2.get();
  //  coralSensor1Output =getCoralSensor1();
  /* coralSensor2Output =getCoralSensor2();
    
    if(coralSensor2Output == false){
      hasCoral =true;
    }
    else{
      hasCoral = false;
    }
    */
 //   backCoralSensorPublisher.setDefault(false);
 //   backCoralSensorPublisher.set(coralSensor3Output);
  }
}