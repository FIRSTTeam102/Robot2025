// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private SparkClosedLoopController climberClosedLoopController = climberMotor.getClosedLoopController(); 
  private SparkMaxConfig climberConfig = new SparkMaxConfig();

  private static boolean isOut = false;
  private SparkMax funnelMotor = new SparkMax(Constants.ClimberConstants.funnelMotorID, MotorType.kBrushless);
  private static boolean funnelOut = false; //checks if funnel is out


  
  PIDController climberPID = new PIDController(Constants.ClimberConstants.climberKp, Constants.ClimberConstants.climberKi, Constants.ClimberConstants.climberKd);
  PIDController funnelPID = new PIDController(Constants.ClimberConstants.funnelKp, Constants.ClimberConstants.funnelKi, Constants.ClimberConstants.funnelKd);


    public Climber() {
     climberConfig.idleMode(IdleMode.kBrake);
     climberConfig.encoder.positionConversionFactor(125);
     climberConfig
      .closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.ClimberConstants.climberKp)
      .i(Constants.ClimberConstants.climberKi)
      .d(Constants.ClimberConstants.climberKd);     
    }
    
    public void setFunnelPosition(){
      if(isOut){
        
      }
      else{
      }
    }

    public void setClimberPosition(){
      if(isOut){
        climberMotor.set(climberPID.calculate(getClimberEncoderPosition(), Constants.ClimberConstants.climberInPosition));
      }
      else{
        climberMotor.set(climberPID.calculate(getClimberEncoderPosition(), Constants.ClimberConstants.climberOutPosition));
      }
    }
    public void stopClimber(){
      climberMotor.stopMotor();
    }
    
    public void stopFunnel(){
      funnelMotor.stopMotor();
    }
  
  
    public double getClimberEncoderPosition(){
      return 2; //DO NOT USE RIGHT NOW
    }

    public double getFunnelEncoderPosition(){
      return funnelMotor.getAlternateEncoder().getPosition();
      
    }
    public static boolean getisOut(){ //checks if the climber is out or not
      return isOut;
    }
    public void toggleisOut(){ //reverses value of boolean
      isOut = !isOut;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
