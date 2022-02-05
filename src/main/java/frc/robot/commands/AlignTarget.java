package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionProcessing;

public class AlignTarget extends CommandBase {

    private VisionProcessing mVision;
    private Drivetrain mDrivetrain;
    private Shooter mShooter;
    private double angle, turnSpeed;


    public AlignTarget(VisionProcessing subsystemA, Drivetrain subsystemB, Shooter subsystemC){
        mVision = subsystemA;
        mDrivetrain = subsystemB;
        mShooter= subsystemC;
        addRequirements(mVision, mDrivetrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        angle = mVision.getYaw();
        if (angle < -Constants.Vision.turnLowerLimit) {
            turnSpeed = mVision.configTurn(angle);
        }
        else if (angle < -Constants.Vision.turnLowerLimit) {
            turnSpeed = -mVision.configTurn(angle);
        }
        else {
            turnSpeed = 0;
        }
        mDrivetrain.vpDrive(turnSpeed);

    }

    @Override
    public void end(boolean isFinshed){

    }
}
