package frc.robot.limelightvision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveState;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.LimelightVision.*;

public class LimelightAlignCommand extends CommandBase {

    private final Drivetrain mDrivetrain;
    private final VPLimelight mVision;

    private boolean stopFlag;

    private final DriveState limelightMode;

    private final double searchDirection;

    private double startTime, elapsedTime;

    private final double bufferTime = 0.45;

    public LimelightAlignCommand(Drivetrain subsystemA, VPLimelight subsystemB, TurnDirection turn, TurnMode mode){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

//        addRequirements(mDrivetrain, mVision);
        addRequirements(mVision);

        searchDirection = (turn == TurnDirection.LEFT) ? -1: 1;

        limelightMode = (mode == TurnMode.TELEOP) ? DriveState.TELE_LIMELIGHT : DriveState.AUTO_LIMELIGHT;

    }


    @Override
    public void initialize(){
//        mVision.steadyArray();
        mDrivetrain.mState = limelightMode;
        stopFlag = false;
//        speed = 0.5;
        elapsedTime = 0;
        System.out.println("Starting Alignment");
    }

    @Override
    public void execute(){
        mVision.updateTargets();
        if(mVision.getTargets() >= 1) {
            stopFlag = mVision.aimTarget(mDrivetrain, limelightMode, startTime, elapsedTime);
            System.out.println("Aiming at Targets");
        } else {
//            mDrivetrain.printMotors();
            mVision.findTarget(mDrivetrain, searchDirection);
            System.out.println("Finding Targets");
        }
//        System.out.println(mVision.getyOffset());
    }

    @Override
    public void end(boolean isFinished){
//        mVision.offArray();
        if(stopFlag){
            System.out.println("Ending Alignment");
        }
        mDrivetrain.mState = (limelightMode == DriveState.TELE_LIMELIGHT) ? DriveState.TELE_DRIVE_INTAKE : DriveState.AUTO_DRIVE;
    }

    @Override
    public boolean isFinished(){
        return stopFlag;
    }

}


