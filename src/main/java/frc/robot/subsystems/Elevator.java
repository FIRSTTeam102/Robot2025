package frc.robot.subsystems;


import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
    
    private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    
    // initialze PID controller and encoder objects
    private SparkAbsoluteEncoder shaftEncoder = motor.getAbsoluteEncoder();
    
    private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);

 
    // TODO: add lidar sensor (distance)
  
    private SparkMaxConfig config = new SparkMaxConfig(); //encoder!!!

   public Elevator(){

    //each revolution of the encoder is 2*3.14*3 =6.283
    //sprocket gear is 2" diameter & we have a 3 stage elevator
       config.encoder
        .positionConversionFactor(ElevatorConstants.conversionFactor_inches_per_roatation)
        .velocityConversionFactor(ElevatorConstants.conversionFactor_ips_per_rpm);

       config   
        .idleMode(IdleMode.kBrake);
    //Set PID values
       config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
       config.closedLoop.iZone(ElevatorConstants.kIz);
       config.closedLoop.minOutput(ElevatorConstants.kMinOutput);
       config.closedLoop.maxOutput(ElevatorConstants.kMaxOutput);

    }
    
@AutoLogOutput
private double motorSpeedOutput = motor.getAppliedOutput();

double height = 0; 

public void manualMove(double motorSpeed){
    closedLoopController.setReference(motorSpeed, ControlType.kVelocity);
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

    closedLoopController.setReference(height, ControlType.kPosition);
}
//TODO if the botom limit switch is triggered zero the encoder
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
  if (bottomlimitSwitch.get() == true){
    zeroEncoder();
  }
}

}