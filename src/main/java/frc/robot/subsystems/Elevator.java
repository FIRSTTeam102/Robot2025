package frc.robot.subsystems;


import java.beans.Encoder;

import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    // create the DCMotor objects to specify the motor type
    DCMotor maxGearbox = DCMotor.getNEO(1);

    private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    // create the Spark MAX sim object
    SparkMaxSim motorSim = new SparkMaxSim(motor, maxGearbox);
    private ElevatorSim m_elevatorSim = null;

    private final RelativeEncoder encoder = motor.getAlternateEncoder();
    

    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    
    private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);
    private boolean prevSetEncoder = true;
 
    // TODO: add lidar sensor (distance)

    private SparkMaxConfig config = new SparkMaxConfig();

   public Elevator(){

        
    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
      config.idleMode(IdleMode.kBrake);

      config.encoder.positionConversionFactor(1);

     // config.closedLoop
      config.closedLoopRampRate(0.5);
        //tell the pid loop how fast to move the motor to achieve position goal
       /**  .maxMotion
          .maxVelocity(ElevatorConstants.motor_max_rpm)
          .maxAcceleration(ElevatorConstants.motor_max_accel)
          .allowedClosedLoopError(ElevatorConstants.ElevatorDefToleranceRotations); */
        //set the feedback sensor to the alternate encoder - rev shaft encoder plugged in
        //to the sparkmax via the alternate encoder adaptor
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)

        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.5)
        .i(0.00009)
        .d(0)
        .velocityFF(ElevatorConstants.kFF)
        .outputRange(-1*ElevatorConstants.maxHeight_rotations,ElevatorConstants.maxHeight_rotations)
        // Set PID values for velocity control in slot 1
        .p(ElevatorConstants.kP, ClosedLoopSlot.kSlot1)
        .i(ElevatorConstants.kI, ClosedLoopSlot.kSlot1)
        .d(ElevatorConstants.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(0.000015, ClosedLoopSlot.kSlot1)
        .outputRange(ElevatorConstants.kMinOutput,ElevatorConstants.kMaxOutput, ClosedLoopSlot.kSlot1);
        
        //rev throughbore definition
        
        config.alternateEncoder.countsPerRevolution(8192);
                               
        
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
    
    //assume the elevator is in the down position, so zero it out & set prev set to true
    zeroEncoder();
    prevSetEncoder = true;

    
    

    if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(maxGearbox,
                    ElevatorConstants.gear_ratio_to_1,
                    Units.lbsToKilograms(ElevatorConstants.carriageMass),
                    Units.inchesToMeters(2),
                    0,
                    Units.inchesToMeters(ElevatorConstants.maxHeight_inches),
                    true,
                    0.0,
                    0.0,
                    0.0);
    }
  }
    


/*
 * get the current position of the elevator in inches, based on the encoder reading
 */
public double getPositionInches() {
  return (encoder.getPosition() * ElevatorConstants.inches_per_rotation);
}
/*
 * get the current velocity - how fast is the elevator moving in inches per
 *   second
 */
public double getVelocityInchesPerSecond() {
  return (encoder.getVelocity() / 60.0) * ElevatorConstants.inches_per_rotation;
}

/*
 * ManualMove takes the percentage of maximum motor speed (velocity)
 */
public void manualMove(double motorSpeedPercent){
    double motorSpeed = motorSpeedPercent * ElevatorConstants.motor_max_rpm;
    closedLoopController.setReference(motorSpeed, ControlType.kVelocity,ClosedLoopSlot.kSlot1,ElevatorConstants.KV);

}

public void stop() {
    motor.stopMotor();
}
/*
 * MoveToSetPosition - go to the height in inches specified
 */
public void moveToSetPosition (double height) {
    //convert the height in inches to rotations
    double rotationGoal = height * ElevatorConstants.rotations_per_inch;
    //clamp returns a value between the max & min rotations possible for the elevator
    // preventing us from going over the top or through the floor
    closedLoopController.setReference(
            MathUtil.clamp(rotationGoal,0,ElevatorConstants.maxHeight_rotations), 
                     //SparkBase.ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,
                     SparkBase.ControlType.kPosition,ClosedLoopSlot.kSlot0,
                     ElevatorConstants.KV);

}


/*
*  move to Position - create a command that uses the pid loop to set the height
*    in inches
*/
public Command moveToPosition(double height)
  {
    System.out.println("moveToPos, height:"+ height);
    return run(() -> {
      moveToSetPosition(height);

    });
  }
    //set the elevator height until it is close to the right height
    public Command setElevatorHeight(double height){
                return moveToPosition(height).until(()->aroundHeight(height));
    }
    //allow a close enough height estimate on the elevator, defaults to constant but
    //can be overridden
    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.ElevatorDefaultToleranceInch);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionInches(),tolerance);
    }
    //make if st atement to add hold stil here when is true

//if the bottom limit switch is triggered zero the encoder
public void zeroEncoder() {
  encoder.setPosition(0.0);
  prevSetEncoder = true;
}
//adjust the output voltage to hold the elevator in place //TODO tune this
public void holdStill(){
  motor.setVoltage(ElevatorConstants.OUTPUT_VOLTS);
}

public Command stayStill(){
  return run(() -> {
    holdStill(); 
  });
}
public boolean bottomLimitSwitchIsBeingPressed() {
  //when this limit switch is being pressed it returns low &
  //returns high when not being pressed, so return the opposite.
  return (!bottomlimitSwitch.get());
}
@AutoLogOutput
    private double voltageLogged; 
@AutoLogOutput
    private double currPositon;
@AutoLogOutput
    private double currVelInchesPerSec;
@AutoLogOutput
    private double currRotations;
@AutoLogOutput
    private double AppliedOutput;
@AutoLogOutput
    private double OutputCurrent;

@Override
public void periodic()
{
  //zero the encode when we trigger the bottom limit switch
  voltageLogged = motor.getAppliedOutput();
  currPositon = getPositionInches();
  currVelInchesPerSec = getVelocityInchesPerSecond();
  currRotations = encoder.getPosition();
  OutputCurrent = motor.getOutputCurrent();

 // System.out.println("value of limit swtich" + bottomlimitSwitch.get());

    //sets position(inces) to 0 if bottomLimistswitch is triggered 
    if(bottomLimitSwitchIsBeingPressed() && !prevSetEncoder){
      System.out.println("reset height to 0");
      zeroEncoder();
    }
    if (!bottomLimitSwitchIsBeingPressed()){
      prevSetEncoder = false;
    }
}
public void simulationPeriodic() {
  //set input(voltage)
  m_elevatorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

  //update-every 20 milliseconds
  m_elevatorSim.update(0.02);
  double velocityMeterPerSec = m_elevatorSim.getVelocityMetersPerSecond();
  double simRotations = Units.metersToInches(velocityMeterPerSec * 60.0) * ElevatorConstants.rotations_per_inch;
  motorSim.iterate(simRotations,
          RoboRioSim.getVInVoltage(),
          0.020);

  RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));


 
}
}