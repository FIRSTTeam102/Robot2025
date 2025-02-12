package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.hal.DIOJNI;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.parser.json.modules.ConversionFactorsJson;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

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


