package frc.robot.subsystems.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class Elevator {
    private SparkMax motor = new SparkMax( 0, MotorType.kBrushless); //change ID later
    private SparkClosedLoopController closedLoopController = motor.getClosedLoopController(); 
    private RelativeEncoder encoder = motor.getEncoder();
    public Elevator(){

    }


float height = 0; 
public void moveManual (){

}
public void moveUp (float height) {
    
}}