package frc.robot.limelightvision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.MathEqs;

import static frc.robot.Constants.LimelightVision.*;

public class LimelightAlignShooterCommand extends CommandBase {

    /***
     * WIP class for adjusting distance and angle
     * Now LimelightAlignCommand is being used
     */

    private Drivetrain mDrivetrain;
    private Indexer mIndexer;
    private Shooter mShooter;
    private VPLimelight mLimelight;

    private double start, elapsedTime;
    private double yOffset, distance, shooterSpeed;
    private double lowHoodRange = 80; //Placeholder
    private double highHoodRange= 120; //^^
    private double minRange = 20; //^^
    private double maxRange = 250; //^^

    public LimelightAlignShooterCommand(Drivetrain subsystemA, Indexer subsystemB, Shooter subsystemC, VPLimelight subsystemD){
        mDrivetrain = subsystemA;
        mIndexer = subsystemB;
        mShooter = subsystemC;
        mLimelight = subsystemD;
        addRequirements(mDrivetrain, mIndexer, mShooter, mLimelight);
    }

    @Override
    public void initialize(){
        mLimelight.updateTargets();
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
        shooterSpeed = calcShooterSpeed();
        mShooter.setShooterVel(shooterSpeed);
        start = System.currentTimeMillis();
        mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_LIMELIGHT;
    }

    @Override
    public void execute(){
        mLimelight.updateTargets();
        if(mLimelight.getTargets() >= 1) {
            mLimelight.steadyArray();
//            aimTarget();
//            System.out.println("Aiming at Targets");
        } else {
            mDrivetrain.printMotors();
//            findTarget();
//            System.out.println("Finding Targets");
        }
        elapsedTime = System.currentTimeMillis() - start;
        mShooter.setShooterVel(shooterSpeed);
        if (elapsedTime > 0.75){
            mIndexer.feedIndexer();
        }
    }

    @Override
    public void end(boolean isFinished){
        mShooter.setShooterIdle();
        mIndexer.setIndexerIdle();
        mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
    }



    public double calcShooterSpeed(){
        distance = mLimelight.calcDistance();

        System.out.println("Shooter Velocity: " + shooterSpeed);
        return shooterSpeed;
    }
}
