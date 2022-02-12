package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;

public class CompleteVisionAlign extends SequentialCommandGroup {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private AlignTargetLimeLight mAlignTarget;
    private CorrectDistanceLimelight mCorrectDistance;


    public CompleteVisionAlign(Drivetrain subsystemA, VPLimelight subsystemB, AlignTargetLimeLight commandA, CorrectDistanceLimelight commandB){
        mDrivetrain = subsystemA;
        mVision = subsystemB;
        addRequirements(mDrivetrain, mVision);


        addCommands(mAlignTarget, mCorrectDistance);

    }



}
