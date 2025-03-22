// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.DrivebaseConstants.TargetSide;

/** helper class to keep track of our current reef target
 *  this is a singleton class that will be constantly updated with our current
 *  target id.
 * . */

public class SharedData {
    private static SharedData instance;
    private int targetAprilTagID = 0;

    public long lastPrintTimestamp = 0;
    public int delay = 1*1000;
    
    private SharedData(){
    }
    /*
     * return or create the instance of this class
     */
    public static SharedData getInstance()
    {
        if (instance == null) {
            synchronized (SharedData.class) {
                if (instance == null) {
                    instance = new SharedData();
                }
            }
        }
        return instance;
    }     
    /*
     * return the current Tag ID
     */
    public int getCurrentTagID(){
        //TODO - REMOVE Testing only
       // if (targetAprilTagID == 0){return 19;}
        return(targetAprilTagID);
    }
    /*
     * set the current tag id
     */
    public void setCurrentTagID(int tagID){
    
        targetAprilTagID = tagID;
    }
    
}
