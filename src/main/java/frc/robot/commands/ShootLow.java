package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootLow extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;
    private Constants.DriveTrain.DriveState mState;

    private double speed, start, elapsedTime;

    public ShootLow(Shooter subsystemA, Indexer subsystemB, Constants.DriveTrain.DriveState mDriveState){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        mState = mDriveState;
        addRequirements(mShooter, mIndexer);

        speed = -0.38;
    }

    @Override
    public void initialize(){
        mShooter.setAnglerLow();
        mShooter.setShooterVel(speed);
        start = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        elapsedTime = (System.currentTimeMillis() - start) / 1000;
        mShooter.setShooterVel(speed);
        if (mState == Constants.DriveTrain.DriveState.AUTO_DRIVE || mState == Constants.DriveTrain.DriveState.AUTO_LIMELIGHT){
            if (elapsedTime > 1) mIndexer.feedIndexer();
//            if (elapsedTime > 1.5) end(true);
        }


    }

    @Override
    public boolean isFinished(){
        return elapsedTime > 1.5;
    }

    @Override
    public void end(boolean isFinished){
        mShooter.setShooterIdle();
        mIndexer.setIndexerIdle();
    }
}
