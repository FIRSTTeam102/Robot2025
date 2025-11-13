package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.AnimationTypes;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Roboto extends TimedRobot {

    private NetworkTable table;

    @Override 
    public void robotInit() {
        table =  NetworkTableInstance.getDefault().getTable("SmartDashboard");
    } 

    @Override
    public void teleopPeriodic(){
        double testVariable = table.getEntry("Testing").getDouble(0.0);
        System.out.println(testVariable);
    }
    
    

    }

