package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.parser.json.modules.ConversionFactorsJson;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

public class Elevator {
    
        private SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless); //change ID later
    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    // add lidar sensor (distance)
    private SparkMaxConfig config = new SparkMaxConfig(); 
   // private Encoder encoder = new Encoder(null, null); <-- might not be needed
    private DigitalInput bottomlimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT); //change channel later

    public Elevator(){
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000); //change factors later and read documentation on this
        config
            .idleMode(IdleMode.kBrake);

    
    }

float setVelocity = 1; //velocity in rpm (will probably be higher) 
float height = 0; // velocity in rpm for diff lvls


    public void setMotorSpeed(){
// Set the setpoint of the PID controller in raw position mode
    closedLoopController.setReference(setVelocity, ControlType.kPosition);    
}
    


public void moveManual(){
    if ( bottomlimitSwitch.get() == false ) {
        setMotorSpeed();   
    }
}

public void moveToSetPosition (float height) {
    closedLoopController.setReference(height, ControlType.kPosition);
}
}


