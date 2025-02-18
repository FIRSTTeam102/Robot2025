package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    // create the DCMotor objects to specify the motor type
    DCMotor maxGearbox = DCMotor.getNEO(1);

    private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    // create the Spark MAX sim object
    SparkMaxSim maxSim = new SparkMaxSim(motor, maxGearbox);

    private final RelativeEncoder encoder = motor.getAlternateEncoder();

    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    
    private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);

 
    // TODO: add lidar sensor (distance)
  
    private SparkMaxConfig config = new SparkMaxConfig(); 

  

   public Elevator(){

        
    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
      config.idleMode(IdleMode.kBrake);
    
      config.closedLoop
        //set the feedback sensor to the alternate encoder - rev shaft encoder plugged in
        //to the sparkmax via the alternate encoder adaptor
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)

        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.00025)
        .i(0)
        .d(0)
        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput)
        // Set PID values for velocity control in slot 1
        .p(ElevatorConstants.kP, ClosedLoopSlot.kSlot1)
        .i(ElevatorConstants.kI, ClosedLoopSlot.kSlot1)
        .d(ElevatorConstants.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(ElevatorConstants.kFF, ClosedLoopSlot.kSlot1)
        .outputRange(ElevatorConstants.kMinOutput,ElevatorConstants.kMaxOutput, ClosedLoopSlot.kSlot1);
        
        config.alternateEncoder.positionConversionFactor(ElevatorConstants.conversionFactor_inches_per_roatation)
                               .countsPerRevolution(8192);
        
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    


/*
 * get the current position of the elevator in inches, based on the encoder reading
 */
public double getPositionInches() {
  return encoder.getPosition() * ElevatorConstants.conversionFactor_inches_per_roatation;
}
/*
 * get the current velocity - how fast is the elevator moving in inches per
 *   second
 */
public double getVelocityInchesPerSecond() {
  return (encoder.getVelocity() / 60) * ElevatorConstants.conversionFactor_inches_per_roatation;
}

/*
 * ManualMove takes the percentage of maximum motor speed (velocity)
 */
public void manualMove(double motorSpeedPercent){
    double motorSpeed = motorSpeedPercent * ElevatorConstants.maxRPM;
    closedLoopController.setReference(motorSpeed, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
}

public void stop() {
    motor.stopMotor();
}
/*
 * MoveToSetPosition - go to the height in inches specified
 */
public void moveToSetPosition (double height) {
    //the height in inches is in the the encoder position conversion factor
    
    closedLoopController.setReference(
            MathUtil.clamp(height,0,ElevatorConstants.maxHeight_inches), 
            ControlType.kPosition,ClosedLoopSlot.kSlot0);
}
//if the bottom limit switch is triggered zero the encoder
public void zeroEncoder() {
  encoder.setPosition(0);
}

/*
*  move to Position - create a command that uses the pid loop to set the height
*    in inches
*/
public Command moveToPosition(double height)
  {
    return run(() -> {
      moveToSetPosition(height);
    });
  }
    //set the elevator height until it is close to the right height
    public Command setElevatorHeight(double height){
        return moveToPosition(height).until(()->aroundHeight(height));
    }
    //allow a close enough height estimate on the elevator
    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.ElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionInches(),tolerance);
    }

@AutoLogOutput
    private double voltageLogged; 
@AutoLogOutput
    private double motorSpeedOutput;
@AutoLogOutput
    private double currPositon;
@AutoLogOutput
    private double currVelInchesPerSec;

@Override
public void periodic()
{
  //zero the encode when we trigger the bottom limit switch
  motorSpeedOutput= motor.getAppliedOutput();
  voltageLogged = motor.getAppliedOutput();
  currPositon = getPositionInches();
  currVelInchesPerSec = getVelocityInchesPerSecond();

  if (bottomlimitSwitch.get() == true){
    zeroEncoder();
  }
}

}