package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.limelightvision.VPLimelight;

public class ShootCustom extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;
    private VPLimelight mVision;

    public ShootCustom(Shooter subsystemA, Indexer subsystemB, VPLimelight subsystemC){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        mVision = subsystemC;
        addRequirements(mShooter, mIndexer, mVision);
    }

    @Override
    public void initialize(){
//        mShooter.setAnglerLow();
//        mShooter.setShooterClose();
    }

    @Override
    public void execute(){
//        mShooter.setShooterClose();
//        if(mShooter.getShooterVel() > mShooter.getShooterCloseVel()){
//            mIndexer.feedIndexer();
//        }
    }

    @Override
    public void end(boolean isFinished){
//        mIndexer.setIndexerIdle();
//        mShooter.setShooterIdle();
    }

}

