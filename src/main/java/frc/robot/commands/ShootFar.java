package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootFar extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;

    public ShootFar(Shooter subsystemA, Indexer subsystemB){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        addRequirements(mShooter, mIndexer);
    }

    @Override
    public void initialize(){
        mShooter.setAnglerHigh();
        mShooter.setShooterFar();
    }

    @Override
    public void execute(){
        mShooter.setShooterFar();
        if (mShooter.getShooterVel() > mShooter.getShooterFarVel()){
            mIndexer.feedIndexer();
        }

    }

    @Override
    public void end(boolean isFinished){
        mIndexer.setIndexerIdle();
        mShooter.setShooterIdle();
    }



}
