// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Lights extends SubsystemBase{
  private CANdle candle;
  private int  totalLEDS = 68;
  private boolean animateDir = false;
  public enum AnimationTypes {
    BLUE,
    RED,
    WHITE,
    CORAL,
    LEVEL4,
    ALGAE,
    Rainbow,
    USA,
  }
  private AnimationTypes prevAnimation = AnimationTypes.Rainbow;

  public Lights(){
    candle = new CANdle(6);
    // Configure the CANdle
    CANdleConfiguration config = new CANdleConfiguration();
      config.brightnessScalar = 0.5;
      config.stripType = LEDStripType.GRB;
    candle.configAllSettings(config);

  }
 
  public void setPattern(AnimationTypes animation){
    if (prevAnimation != animation) {
    prevAnimation = animation;
    Animation toAnimate = null;
     switch (animation) {
      case LEVEL4:
        toAnimate = new LarsonAnimation(255,30,0,0, 0.75,68, LarsonAnimation.BounceMode.Back,9, 8);
        break;
      case RED:
        toAnimate = new TwinkleAnimation(255, 0, 0, 0, 0.75, 68, TwinkleAnimation.TwinklePercent.Percent88, 0 );
        break;
      case BLUE:
        toAnimate = new TwinkleAnimation(0, 0, 255, 0, 0.75, 68, TwinkleAnimation.TwinklePercent.Percent88, 0 );
        break;
      case WHITE:
        toAnimate = new TwinkleAnimation(255, 255, 255, 0, 0.75, 68, TwinkleAnimation.TwinklePercent.Percent88, 0 );
        break;
      case ALGAE:
        toAnimate = new ColorFlowAnimation(0, 255, 0, 0, 0.5, 68, ColorFlowAnimation.Direction.Forward, 0);
        break;
      case CORAL:
        toAnimate = new ColorFlowAnimation(255, 255, 255, 0, 0.5, 68, ColorFlowAnimation.Direction.Forward, 0);
        break;
      case USA:
        toAnimate = new TwinkleAnimation(60, 0, 0, 0,0.7, 27,TwinkleAnimation.TwinklePercent.Percent76, 0);
        toAnimate = new TwinkleAnimation(60, 60, 60, 60,0.7, 20,TwinkleAnimation.TwinklePercent.Percent76, 28);
        toAnimate = new TwinkleAnimation(0, 0, 60, 60,0.7, 20,TwinkleAnimation.TwinklePercent.Percent76, 48);
        break;
      case Rainbow:
        toAnimate = new RainbowAnimation(1,0.7,totalLEDS);
        break;
      default:
        toAnimate = new RainbowAnimation(1,0.7,totalLEDS);
    }
      candle.animate(toAnimate,0);   
    }
  }
  public void setForAllianceDefault(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
      setPattern(AnimationTypes.RED);
    }
    else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue){
      setPattern(AnimationTypes.BLUE);
    }
    else {
      setPattern(AnimationTypes.USA);
    }
  }
}
