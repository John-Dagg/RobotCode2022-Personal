package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;

public class CompleteVisionAlign extends SequentialCommandGroup {



    private AlignTargetLimeLight mAlignTarget;
    private CorrectDistanceLimelight mCorrectDistance;


    public CompleteVisionAlign(AlignTargetLimeLight commandA, CorrectDistanceLimelight commandB){

        mAlignTarget = commandA;
        mCorrectDistance = commandB;


        addCommands(mAlignTarget, mCorrectDistance);

    }



}
