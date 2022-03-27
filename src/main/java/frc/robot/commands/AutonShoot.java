package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private double bufferTime = 1.0;
    private boolean robotAligned;

    public AutonShoot(Drivetrain subsystemA, Intake subsystemB, Indexer subsystemC, Shooter subsystemD, VPLimelight subsystemE,
                      TurnDirection turnDirection, HoodState hoodState, IntakeState intakeState, double speed, double time){

        mDrivetrain = subsystemA;mIntake = subsystemB;mIndexer = subsystemC; mShooter = subsystemD; mLimelight = subsystemE;
        mTurnDirection = turnDirection;
        mHoodState = hoodState;
        mIntakeState = intakeState;

        this.speed = speed;
        this.time = time;
    }

    @Override
    public void initialize(){
        start = System.currentTimeMillis();

        turnDir = mTurnDirection == TurnDirection.LEFT ? 1.0 : -1.0;
        if (mHoodState == HoodState.HIGH) mShooter.setAnglerHigh(); else mShooter.setAnglerLow();
        if (mIntakeState == IntakeState.IN) mIntake.retractIntake(); else mIntake.extendIntake();

        mShooter.setShooterVel(speed);

        mDrivetrain.mState = AUTO_LIMELIGHT;
        robotAligned = false;
        elapsedTime = 0;
        startTime = -1;
    }

    @Override
    public void execute(){
        elapsedTime = (System.currentTimeMillis() - start) /1000;

        if (mTurnDirection != TurnDirection.NONE) {
            mLimelight.updateTargets();
        }
        else {
            robotAligned = true;
        }

        if (mLimelight.getTargets() >= 1) {
            robotAligned = mLimelight.aimTarget(mDrivetrain, AUTO_LIMELIGHT, startTime, bufferTime);
        } else {
            mLimelight.findTarget(mDrivetrain, turnDir);
        }

        mShooter.setShooterVel(speed);
        if (robotAligned && Math.abs(mShooter.getShooterVel()) > 0.55){
            System.out.println(mShooter.getShooterVel());
            mIndexer.feedIndexer();
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
