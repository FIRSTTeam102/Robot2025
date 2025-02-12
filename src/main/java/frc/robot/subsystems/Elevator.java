package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
    
    private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless); //change ID later
    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
  // TODO: add lidar sensor (distance)
  private SparkMaxConfig config = new SparkMaxConfig(); //encoder!!!
  // private Encoder encoder = new Encoder(null, null); <-- might not be needed
   private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT); //change channel later

   public Elevator(){
       config.encoder
        .positionConversionFactor(ElevatorConstants.conversionFactor_m_per_roatation)
        .velocityConversionFactor(ElevatorConstants.conversionFactor_mps_per_rpm); //TODO: change factors later

       config   
        .idleMode(IdleMode.kBrake);
        //TODO: add ramp rate (maybe) and change PID values
       config.closedLoop
       .pid(1, 0, 0);
    }
    
@AutoLogOutput
private double motorSpeedOutput = motor.getAppliedOutput();

double height = 0; 

public void manualMove(double motorSpeed){
    closedLoopController.setReference(motorSpeed, ControlType.kDutyCycle); //notsure which way to move motors I should use

}

public void stop() {
    closedLoopController.setReference(0, ControlType.kDutyCycle);
}

public void moveToSetPosition (double height) {
    closedLoopController.setReference(height, ControlType.kPosition);
}
}


