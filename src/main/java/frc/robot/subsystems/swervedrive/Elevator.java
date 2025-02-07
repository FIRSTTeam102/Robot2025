package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.parser.json.modules.ConversionFactorsJson;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

public class Elevator {
    private SparkMax motor = new SparkMax( 0, MotorType.kBrushless); //change ID later
    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
  // TODO: add lidar sensor (distance)
  private SparkMaxConfig config = new SparkMaxConfig(); //encoder!!!
  // private Encoder encoder = new Encoder(null, null); <-- might not be needed
   private DigitalInput bottomlimitSwitch = new DigitalInput(1); //change channel later

   public Elevator(){
       config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000); //TODO: change factors later
    }
    


float height = 0; 

public class ManualElevatorControl extends CommandBase {
    private Elevator elevator;
    private DoubleSupplier inputSupplier;
    
    public void moveManual (Elevator elevator, DoubleSupplier inputSupplier){
    this.elevator = elevator;
    this.inputSupplier = inputSupplier;
    addRequirements(elevator);
    elevator.setManualModeInput(inputSupplier);
    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (elevator.inManualMode)
            elevator.setSpeed(
                MathUtil.applyDeadband(inputSupplier.getAsDouble(), OperatorConstants.operatorJoystickDeadband)*-.4);
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
public void moveUp (float height) {
    
}}