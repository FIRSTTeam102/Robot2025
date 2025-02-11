package frc.robot.subsystems.swervedrive;

import frc.robot.Constants.DrivebaseConstants.TargetSide;;

//Measurements are in meters and match what path planner shows for x, y and angle
//TODO align to targets on physical field & determine actual pose
 public final class ScoringPositionConstants {
    public static final ScoringPosConst scoringPositions[] = {
		//RED SIDE REEF TARGETS
		new ScoringPosConst(6, TargetSide.LEFT, 13.72, 2.45, 127),
		new ScoringPosConst(6,TargetSide.RIGHT,14.0,2.67, 127),
		new ScoringPosConst(7, TargetSide.LEFT, 14.7, 3.8, 0),
		new ScoringPosConst(7,TargetSide.RIGHT,14.642,4.444, 0),
		new ScoringPosConst(8, TargetSide.LEFT, 14.1, 5.3, 53),
		new ScoringPosConst(8,TargetSide.RIGHT,13.7,5.57, 53),
		new ScoringPosConst(9, TargetSide.LEFT, 11.9, 5.33, 96),
		new ScoringPosConst(9,TargetSide.RIGHT,12.48,5.5, 96),
		new ScoringPosConst(10, TargetSide.LEFT, 11.5, 3.8, 0),
		new ScoringPosConst(10,TargetSide.RIGHT,14.156,2.766, 0),
		new ScoringPosConst(11, TargetSide.LEFT, 11.9, 2.7, 52),
		new ScoringPosConst(11,TargetSide.RIGHT,12.5,2.5, 52),

		//BLUE SIDE REEF TARGETS
		new ScoringPosConst(17, TargetSide.LEFT, 3.321, 2.77, 35),
		new ScoringPosConst(17,TargetSide.RIGHT,3.829,2.48, 35),
		new ScoringPosConst(18, TargetSide.LEFT, 2.8, 4.289, 0),
		new ScoringPosConst(18,TargetSide.RIGHT,2.8,3.741, 0),
		new ScoringPosConst(19, TargetSide.LEFT, 3.819, 5.576, -58.173),
		new ScoringPosConst(19,TargetSide.RIGHT,3.47,5.396, -58.173),
		new ScoringPosConst(20, TargetSide.LEFT, 5.724, 5.237, -125.628),
		new ScoringPosConst(20,TargetSide.RIGHT,5.185,5.556,-125.628),
		new ScoringPosConst(21, TargetSide.LEFT, 6.143, 4.18, 180),
		new ScoringPosConst(21,TargetSide.RIGHT,14.156,2.766, 180),
		new ScoringPosConst(22, TargetSide.LEFT, 5.125, 2.524, 153),
		new ScoringPosConst(22,TargetSide.RIGHT,5.5,2.7, 153)
		

	};
 }

 