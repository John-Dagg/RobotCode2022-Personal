package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootClose extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;

    double start, time, elapsedTime;

    public ShootClose(Shooter subsystemA, Indexer subsystemB, double time){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        this.time = time * 1000;
        addRequirements(mShooter, mIndexer);
    }

    public ShootClose(Shooter subsystemA, Indexer subsystemB){
        this(subsystemA, subsystemB, 1000000000);
    }

    @Override
    public void initialize(){
        start = System.currentTimeMillis();

        mShooter.setAnglerLow();
        mShooter.setShooterClose();

    }

    @Override
    public void execute(){
        elapsedTime = System.currentTimeMillis() - start;
        mShooter.setShooterClose();
        if(elapsedTime > 0.5){
            mIndexer.feedIndexer();
        }
    }

    @Override
    public boolean isFinished(){
        return elapsedTime > time;
    }

    @Override
    public void end(boolean isFinished){
        mIndexer.setIndexerIdle();
        mShooter.setShooterIdle();
    }


}
