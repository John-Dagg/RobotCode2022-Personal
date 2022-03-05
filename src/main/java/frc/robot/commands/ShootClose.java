package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootClose extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;

    public ShootClose(Shooter subsystemA, Indexer subsystemB){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        addRequirements(mShooter, mIndexer);
    }

    @Override
    public void initialize(){
        mShooter.setAnglerLow();
        mShooter.setShooterClose();
    }

    @Override
    public void execute(){
        mShooter.setShooterClose();
        if(mShooter.getShooterVel() > mShooter.getShooterCloseVel()){
            mIndexer.feedIndexer();
        }
    }

    @Override
    public void end(boolean isFinished){
        mIndexer.setIndexerIdle();
        mShooter.setShooterIdle();
    }


}
