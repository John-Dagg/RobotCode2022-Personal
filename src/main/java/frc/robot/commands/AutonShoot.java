package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.DriveTrain.DriveState.AUTO_DRIVE;
import static frc.robot.Constants.DriveTrain.DriveState.AUTO_LIMELIGHT;
import static frc.robot.Constants.LimelightVision.TurnDirection;
import static frc.robot.Constants.IntakeState;
import static frc.robot.Constants.HoodState;

public class AutonShoot extends CommandBase {

    private Drivetrain mDrivetrain;
    private Intake mIntake;
    private Indexer mIndexer;
    private Shooter mShooter;
    private VPLimelight mLimelight;

    private TurnDirection mTurnDirection;
    private HoodState mHoodState;
    private IntakeState mIntakeState;

    private double speed, time, elapsedTime, start, startTime, turnDir;
    private boolean robotAligned, customShootTime;
    private double shootTime, endTime;

    public AutonShoot(Drivetrain subsystemA, Intake subsystemB, Indexer subsystemC, Shooter subsystemD, VPLimelight subsystemE,
                      TurnDirection turnDirection, HoodState hoodState, IntakeState intakeState, double speed, double time){

        mDrivetrain = subsystemA;mIntake = subsystemB;mIndexer = subsystemC; mShooter = subsystemD; mLimelight = subsystemE;
        mTurnDirection = turnDirection;
        mHoodState = hoodState;
        mIntakeState = intakeState;

        this.speed = speed;
        this.time = time;
        customShootTime = false;
    }

    public AutonShoot(Drivetrain subsystemA, Intake subsystemB, Indexer subsystemC, Shooter subsystemD, VPLimelight subsystemE,
                      TurnDirection turnDirection, HoodState hoodState, IntakeState intakeState,
                      double speed, double shootTime, double endTime){

        mDrivetrain = subsystemA;mIntake = subsystemB;mIndexer = subsystemC; mShooter = subsystemD; mLimelight = subsystemE;
        mTurnDirection = turnDirection;
        mHoodState = hoodState;
        mIntakeState = intakeState;

        this.speed = speed;
        this.shootTime = shootTime;
        this.endTime = endTime;
        customShootTime = true;
    }

    @Override
    public void initialize(){
        start = System.currentTimeMillis();

        turnDir = mTurnDirection == TurnDirection.LEFT ? 1.0 : -1.0;
        if (mHoodState == HoodState.HIGH) mShooter.setAnglerHigh(); else mShooter.setAnglerLow();
        if (mIntakeState == IntakeState.IN) mIntake.retractIntake(); else mIntake.extendIntake();

        mShooter.setShooterGain(speed);

        mDrivetrain.mState = AUTO_LIMELIGHT;
        robotAligned = false;
        elapsedTime = 0;
        startTime = -1;

        time = endTime;
    }

    @Override
    public void execute(){
        elapsedTime = (System.currentTimeMillis() - start) /1000;

        mLimelight.updateTargets();

        if (mTurnDirection == Constants.LimelightVision.TurnDirection.NONE) {
            robotAligned = true;
        }

        if (mLimelight.getTargets() >= 1 && !robotAligned) {
            robotAligned = mLimelight.aimTarget(mDrivetrain, AUTO_LIMELIGHT);
        } else if (!robotAligned){
            mLimelight.findTarget(mDrivetrain, turnDir);
        }

        mShooter.setShooterGain(speed);
        System.out.println("SHOOTER VEL: " + mShooter.getShooterVel());
        if (customShootTime) {
            if (elapsedTime > shootTime && Math.abs(mShooter.getShooterVel()) > Math.abs(speed) - 0.1) {
                mIndexer.feedIndexer();
            }
        } else {
            if (elapsedTime > 1 && Math.abs(mShooter.getShooterVel()) > Math.abs(speed) - 0.1) {
                mIndexer.feedIndexer();
            }
        }

    }

    @Override
    public void end(boolean isFinished){
        mShooter.setShooterIdle();
        mIndexer.setIndexerIdle();
        mDrivetrain.mState = AUTO_DRIVE;
    }

    @Override
    public boolean isFinished(){
        return elapsedTime > time;
    }

}
