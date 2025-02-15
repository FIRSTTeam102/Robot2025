package frc.robot.subsystems;


import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    // create the DCMotor objects to specify the motor type
    DCMotor maxGearbox = DCMotor.getNEO(1);

    private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    // create the Spark MAX sim object
    SparkMaxSim maxSim = new SparkMaxSim(motor, maxGearbox);

    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    
    // initialze PID controller and encoder objects
   // private SparkAbsoluteEncoder shaftEncoder = motor.getAbsoluteEncoder();
    
    private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);

 
    // TODO: add lidar sensor (distance)
  
    private SparkMaxConfig config = new SparkMaxConfig(); 

    @AutoLogOutput
    private double voltageLogged;

   public Elevator(){

    //each revolution of the encoder is 2*3.14*3 =6.283
    //sprocket gear is 2" diameter & we have a 3 stage elevator
       config.encoder
        .positionConversionFactor(ElevatorConstants.conversionFactor_inches_per_roatation)
        .velocityConversionFactor(ElevatorConstants.max_linear_ips);

       config   
        .idleMode(IdleMode.kBrake);
    
    //Set PID values
    /*   config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
       config.closedLoop.iZone(ElevatorConstants.kIz);

    */

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.25)
        .i(0)
        .d(0)
        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput)
        // Set PID values for velocity control in slot 1
        .p(.15, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(0.01056, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

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
    
@AutoLogOutput
private double motorSpeedOutput;

double height = 0; //inches

public void manualMove(double motorSpeed){
    closedLoopController.setReference(motorSpeed, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
}

public void stop() {
    motor.stopMotor();
}
/*
 * MoveToSetPosition - go to the height in inches specified
 */
public void moveToSetPosition (double height) {
       //if the requested height is less than zero - go to zero
    //if the requested height is higher than the max, go to the max
    if (height < 0) height = 0;
    if (height > ElevatorConstants.maxHeight_inches) height = ElevatorConstants.maxHeight_inches;

    closedLoopController.setReference(height, ControlType.kPosition,ClosedLoopSlot.kSlot0);
}
//TODO if the bottom limit switch is triggered zero the encoder
public void zeroEncoder() {
}

/*
*  move to Position - create a command that uses the pid loop to set the height
*/
public Command moveToPosition(double height)
  {
    return run(() -> {
      moveToSetPosition(height);
    });
  }

@Override
public void periodic()
{
  //zero the encode when we trigger the bottom limit switch
  motorSpeedOutput= motor.getAppliedOutput();
  voltageLogged = motor.getAppliedOutput();
  if (bottomlimitSwitch.get() == true){
    zeroEncoder();
  }
}

}