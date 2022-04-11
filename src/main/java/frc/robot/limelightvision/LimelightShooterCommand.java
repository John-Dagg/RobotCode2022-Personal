package frc.robot.limelightvision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class LimelightShooterCommand extends CommandBase {

    private Shooter mShooter;
    private Indexer mIndexer;
    private VPLimelight mLimelight;

    private double start, elapsedTime;
    private double yOffset, distance, speed;
    private double lowHoodRange = 80; //Placeholder
    private double highHoodRange= 120; //Placeholder
    private double minRange = 20; //^^
    private double maxRange = 250; //^^
    /**
     *  If lower than the lowHoodRange value than use the low hood angle. Otherwise, use the high hood angle.
     *  It is possible (probably not the case though) that there will be overlap in their ranges.
     *  If so use the high hood angle within the overlap because it may require less velocity and lead to more consistency.
     */

    public LimelightShooterCommand(Shooter subsystemA, Indexer subsystemB, VPLimelight subsystemC){
        mShooter = subsystemA;
        mIndexer = subsystemB;
        mLimelight = subsystemC;
        addRequirements(mShooter, mIndexer, mLimelight);
    }


    //TODO: Gather data to create a formula for the speed based on distance
    public double calcShooterSpeed(){
        distance = mLimelight.calcDistance();

        System.out.println("Shooter Velocity: " + speed);
        return speed;
    }

    @Override
    public void initialize(){
        if (mLimelight.getTargets() < 1){
            System.out.println("No valid Target");
            end(true);
        }
        distance = mLimelight.calcDistance();
        if (distance < lowHoodRange && distance > minRange){
//            mShooter.setAnglerLow();
        } else if (distance > highHoodRange && distance < maxRange){
//            mShooter.setShooterFar();
        } else {
            System.out.println("Out of viable range");
        }
        speed = calcShooterSpeed();
        mShooter.setShooterVel(speed);
        start = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        elapsedTime = System.currentTimeMillis() - start;
        mShooter.setShooterVel(speed);
        if (elapsedTime > 0.75){
            mIndexer.feedIndexer();
        }
    }

    @Override
    public void end(boolean isFinished){
        mShooter.setShooterIdle();
        mIndexer.setIndexerIdle();
    }
}
