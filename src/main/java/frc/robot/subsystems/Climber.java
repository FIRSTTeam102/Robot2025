// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
  private static boolean isOut = false;
  private SparkMax funnelMotor = new SparkMax(Constants.ClimberConstants.funnelMotorID, MotorType.kBrushless);
  private RelativeEncoder climbMotorEncoder = climberMotor.getEncoder();  //encoder for gear conversion.  Found in NEO
  private static boolean funnelOut = false; //checks if funnel is out

    public Climber() {
      climbMotorEncoder.setPositionConversionFactor(125);
        //insert conversions here.
    }
    
    public void setClimberMotorSpeed(double climberMotorSpeed){
      climberMotor.set(climberMotorSpeed);
    }
  
    public void setFunnelMotorSpeed(double funnelMotorSpeed){
      funnelMotor.set(funnelMotorSpeed);
    }
  
    public double getClimberEncoderPosition(){
      return climberMotor.getAbsoluteEncoder().getPosition();
    }

    public double getFunnelEncoderPosition(){
      return funnelMotor.getAbsoluteEncoder().getPosition();
      
    }
    public static boolean getisOut(){ //checks if the climber is out or not
      return isOut;
    }
    public void toggleisOut(){ //reverses value of boolean
      isOut = !isOut;
    }

    public static boolean getfunnelOut(){
      return funnelOut;
    }

    public void togglefunnelOut(){
      funnelOut = !funnelOut;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
