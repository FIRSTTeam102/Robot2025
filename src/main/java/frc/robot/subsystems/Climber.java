package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
    private RelativeEncoder climberEncoder = climberMotor.getEncoder();
    private SparkClosedLoopController climberClosedLoopController = climberMotor.getClosedLoopController();
    private SparkMaxConfig climberConfig = new SparkMaxConfig();
    private static boolean isOut = false;

    private SparkMax funnelMotor = new SparkMax(Constants.ClimberConstants.funnelMotorID, MotorType.kBrushless);
    private RelativeEncoder funnelEncoder = funnelMotor.getEncoder();
    private static boolean funnelOut = false; // Checks if funnel is out
    private SparkClosedLoopController funnelClosedLoopController = funnelMotor.getClosedLoopController();
    private SparkMaxConfig funnelConfig = new SparkMaxConfig();

    public Climber() {
        // Configure climber motor settings
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.encoder.positionConversionFactor(125); // Converts encoder units to real-world values

        // Configure onboard PID with closed-loop control
        climberConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Use integrated NEO encoder
            .p(Constants.ClimberConstants.climberKp)
            .i(Constants.ClimberConstants.climberKi)
            .d(Constants.ClimberConstants.climberKd)
            .outputRange(-1.0, 1.0); // Limit output

        // Apply config and persist settings
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

//Set up funnel motor config

funnelConfig.idleMode(IdleMode.kCoast);
funnelConfig.encoder.positionConversionFactor(3); // Converts encoder units to real-world values

        // Configure onboard PID with closed-loop control
        funnelConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Use integrated NEO encoder
            .p(Constants.ClimberConstants.funnelKp)
            .i(Constants.ClimberConstants.funnelKi)
            .d(Constants.ClimberConstants.funnelKd)
            .outputRange(-Constants.ClimberConstants.maxFunnelRotatations, Constants.ClimberConstants.maxFunnelRotatations); // Limit output

        // Apply config and persist settings
        funnelMotor.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Moves the climber to a set position */
    public void setClimberPosition(double position) {
        climberClosedLoopController.setReference(position,ControlType.kPosition);  
    }

    /** Stops climber motor */
    public void stopClimber() {
        climberMotor.stopMotor();
    }

    /** Stops funnel motor */
    public void stopFunnel() {
        funnelMotor.stopMotor();
    }

    /** Gets current climber position */
    public double getClimberEncoderPosition() {
        return climberEncoder.getPosition();
    }

    /** Gets current funnel position */
    public double getFunnelEncoderPosition() {
        return funnelEncoder.getPosition(); // Ensure correct encoder type
    }

    /** Toggles climber state */
    public void toggleIsOut() {
        isOut = !isOut;
    }

    public void toggleFunnelIsOut() {
      funnelOut = !funnelOut;
  }


    public static boolean getisClimberOut(){
      return isOut;
    }
    
    public static boolean getisFunnelOut(){
      return funnelOut;
    }
    public void setFunnelPosition(double position) {
      funnelClosedLoopController.setReference(position,ControlType.kPosition);  
  }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
