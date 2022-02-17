package frc.robot.photonvision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class PhotonAlignTargetCommand extends CommandBase {
    //Subject for removal due to arrival of Limelight 2.0

    private VPPhoton mVision;
    private Drivetrain mDrivetrain;
    private Shooter mShooter;
    private double angle, turnSpeed;


    public PhotonAlignTargetCommand(VPPhoton subsystemA, Drivetrain subsystemB, Shooter subsystemC){
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
        if (angle < -Constants.PhotonVision.turnLowerLimit) {
            turnSpeed = mVision.configTurn(angle);
        }
        else if (angle < -Constants.PhotonVision.turnLowerLimit) {
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
