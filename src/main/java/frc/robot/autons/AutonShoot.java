package frc.robot.autons;

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

    private double speed, time, elapsedTime, start, startTime, turnDir, shootTime, actualRPM;
    private boolean robotAligned, customShootTime;

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
        this.time = endTime;
        customShootTime = true;
    }

    @Override
    public void initialize(){
        start = System.currentTimeMillis();

        turnDir = mTurnDirection == TurnDirection.LEFT ? 1.0 : -1.0;
//        if (mHoodState == HoodState.HIGH) mShooter.setAnglerHigh(); else mShooter.setAnglerLow();
        if (mIntakeState == IntakeState.IN) mIntake.retractIntake(); else mIntake.extendIntake();

        mShooter.setShooterGain(speed);

        mDrivetrain.mState = AUTO_LIMELIGHT;
        robotAligned = false;
        elapsedTime = 0;
        startTime = -1;
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

        mShooter.PIDshooter(speed);

        actualRPM = mShooter.getShooterRPM();
//        System.out.println("SHOOTER VEL: " + actualRPM);
//        System.out.println(mShooter.checkRPM(speed, actualRPM, 150));
        if (customShootTime) {
            if (elapsedTime > shootTime && mShooter.checkRPM(speed, actualRPM, 150))
                mIndexer.feedIndexer();
        } else {
            if (elapsedTime > 1 && mShooter.checkRPM(speed, actualRPM, 150)) mIndexer.feedIndexer();
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
