package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.limelightvision.VPLimelight;

public class ShootCustom extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;
    private VPLimelight mVision;

    private TalonFX mFalcon;

    private double maxVel = 6000;
    private double hoodAngle = 65;
    private double pitch, dist, vel, percentOutput;

    public ShootCustom(Shooter subsystemA, Indexer subsystemB, VPLimelight subsystemC){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        mVision = subsystemC;
        addRequirements(mShooter, mIndexer, mVision);

        mFalcon = mShooter.getShooterLeader();
    }

    //Inches
    public double calcDistance(){
        pitch = mVision.getyOffset();
        return (Constants.LimelightVision.targetHeight - Constants.LimelightVision.cameraHeight)
                / Math.tan(Units.degreesToRadians(Constants.LimelightVision.cameraAngle + pitch));
    }

    //Inches
    public double calcSpeed(){
        //Equation to find the required initial velocity of a projectile
        //Assumes the ball will reach the same speed as the wheels which is inaccurate sadly
        dist = calcDistance();
        vel = Math.sqrt(((dist * dist) * 9.8)
                / (dist * Math.sin(2 * hoodAngle) - (2 * (Constants.LimelightVision.targetHeight
                - Constants.LimelightVision.cameraHeight) * Math.cos(hoodAngle) * Math.cos(hoodAngle))));
        percentOutput = ((vel / (6 * Math.PI)) / 100 * 4096) / maxVel;
        return vel < 0 ? Math.max(vel, -1.0) : Math.min(vel, 1.0);
    }

    public void setShooterCustom(){

        mFalcon.set(TalonFXControlMode.PercentOutput, calcSpeed());
    }

    public double getShooterCustomVel(){
        return calcSpeed() - 0.1;
    }

    @Override
    public void initialize(){
        mShooter.setAnglerLow();
        setShooterCustom();
    }

    @Override
    public void execute(){
        setShooterCustom();
        if (mShooter.getShooterVel() > getShooterCustomVel()){
            mIndexer.feedIndexer();
        }
    }

    @Override
    public void end(boolean isFinished){
        mIndexer.setIndexerIdle();
        mShooter.setShooterIdle();
    }



}

