package frc.robot.limelightvision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LimelightCompleteVisionCommand extends SequentialCommandGroup {



    private LimelightAlignTargetCommand mAlignTarget;
    private LimelightDistanceCommand mCorrectDistance;


    public LimelightCompleteVisionCommand(LimelightAlignTargetCommand commandA, LimelightDistanceCommand commandB){

        mAlignTarget = commandA;
        mCorrectDistance = commandB;


        addCommands(mAlignTarget, mCorrectDistance);

    }



}
