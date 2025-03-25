// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

/** Add your docs here. */
public class Lights {
  private static Lights instance;
  private CANdle candle;
  private int  totalLEDS = 60;
  private boolean animateDir = false;
  public enum AnimationTypes {
    BLUE,
    RED,
    Empty,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  
}
  private Lights(){
    candle = new CANdle(6);
    // Configure the CANdle
    CANdleConfiguration config = new CANdleConfiguration();
      config.brightnessScalar = 0.5;
      config.stripType = LEDStripType.GRB;
    candle.configAllSettings(config);
  }
  public static Lights getInstance()
  {
      if (instance == null) {
          synchronized (Lights.class) {
              if (instance == null) {
                  instance = new Lights();
              }
          }
      }
      return instance;
  }     
  public void setPattern(AnimationTypes animation){
    Animation toAnimate = null;
    switch (animation) {
      case Rainbow:
        toAnimate = new RainbowAnimation(1, 0.7, totalLEDS, animateDir,8);
        break;
      case RED:
        break;
      case BLUE:
        break;
    }
      candle.animate(toAnimate,1);
    
  }
}
