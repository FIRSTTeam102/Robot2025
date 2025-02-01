package frc.robot.subsystems.swervedrive;

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



public class Elevator {
    private SparkMax motor = new SparkMax( 0, MotorType.kBrushless); //change ID later
    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    // add lidar sensor (distance)
    private SparkMaxConfig config = new SparkMaxConfig(); //encoder!!!
   // private Encoder encoder = new Encoder(null, null); <-- might not be needed
    private DigitalInput bottomlimitSwitch = new DigitalInput(1); //change channel later

    public Elevator(){
        config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000); //change factors later
    }
    


float height = 0; 
public void moveManual (){
    //setMotorSpeed(XboxController.getRawAxis(2));

}
public void moveUp (float height) {
    
}}