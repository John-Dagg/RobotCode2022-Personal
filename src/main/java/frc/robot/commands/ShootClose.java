package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootClose extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;

    double start, time, elapsedTime, speed;
    boolean close;

    public ShootClose(Shooter subsystemA, Indexer subsystemB, double time, double speed, boolean close){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        this.time = time;
        this.speed = speed;
        this.close = close;
        addRequirements(mShooter, mIndexer);
    }

    public ShootClose(Shooter subsystemA, Indexer subsystemB){
        this(subsystemA, subsystemB, 1000000000, -0.65, true);
    }

    @Override
    public void initialize(){
        start = System.currentTimeMillis();
        if (close) mShooter.setAnglerLow();
        else mShooter.setShooterFar();

        mShooter.setShooterVel(speed);

    }

    @Override
    public void execute(){
        elapsedTime = (System.currentTimeMillis() - start) / 1000;
        mShooter.setShooterVel(speed);

        if(elapsedTime > 1 && time != 1000000000){
            mIndexer.feedIndexer();
        }
//        mIndexer.feedIndexer();
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
