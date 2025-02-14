// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
  private static boolean isOut = false;
  private SparkMax funnelMotor = new SparkMax(Constants.ClimberConstants.funnelMotorID, MotorType.kBrushless);
  private static boolean funnelOut = false; //checks if funnel is out
    public Climber() {}
    
    public void setClimberMotorSpeed(double climberMotorSpeed){
      climberMotor.set(climberMotorSpeed);
    }
  
    public void setFunnelMotorSpeed(double funnelMotorSpeed){
      funnelMotor.set(funnelMotorSpeed);
    }
  
    public double getEncoderPosition(){
      return climberMotor.getAbsoluteEncoder().getPosition();
    }
  
    public static boolean getisOut(){ //checks if the climber is out or not
      return isOut;
    }
    public void toggleisOut(){ //reverses value of boolean
      isOut = !isOut;
    }

    public double getFunnelMotorCurrent(){
      return funnelMotor.getOutputCurrent();
    }

    public static boolean getfunnelOut(){
      return funnelOut;
    }

    public void togglefunnelOut(){
      funnelOut = true;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
