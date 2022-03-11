package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootLow extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;

    private double speed, start, elapsedTime;

    public ShootLow(Shooter subsystemA, Indexer subsystemB){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        addRequirements(mShooter, mIndexer);

        speed = 0.3;
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
        if (elapsedTime > 0.25){
            mIndexer.feedIndexer();
        }

    }

    @Override
    public void end(boolean isFinished){
        mShooter.setShooterIdle();
        mIndexer.setIndexerIdle();
    }
}
