package frc.robot.subsystems.swervedrive;
import frc.robot.Constants.DrivebaseConstants.TargetSide;

public record ScoringPosConst(
	int aprilTagId,
	TargetSide scoreSide,
	double xPos,
    double yPos,
    double angleOffset) {
	
	}