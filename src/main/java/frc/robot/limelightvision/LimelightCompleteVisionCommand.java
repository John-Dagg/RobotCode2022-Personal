package frc.robot.limelightvision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LimelightCompleteVisionCommand extends SequentialCommandGroup {



    private LimelightAlignLeftCommand mAlignTarget;
    private LimelightDistanceCommand mCorrectDistance;


    public LimelightCompleteVisionCommand(LimelightAlignLeftCommand commandA, LimelightDistanceCommand commandB){

        mAlignTarget = commandA;
        mCorrectDistance = commandB;


        addCommands(mAlignTarget, mCorrectDistance);

    }



}
