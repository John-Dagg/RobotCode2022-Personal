package frc.robot.limelightvision;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.MathEqs.*;

public class LimelightDistanceCommand extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private LimelightAlignLeftCommand mAlign;

    private CANSparkMax mLeftLeader, mRightLeader;
    private MotorControllerGroup mLeftMotors, mRightMotors;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private double yOffset, speed, goalDistance, initDistance, goalTravel, actualTravel, buffer;

    private final double conversion = 0.1429 * 6 * Math.PI * 2; //Rotations to inches

    private boolean distanceCompleted;
    private boolean stopFlag;

    private final double distanceClose = 62; //inches
    private final double distanceFar = 250; //inches


    public LimelightDistanceCommand(Drivetrain subsystemA, VPLimelight subsystemB, boolean close){
        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftMotors = mDrivetrain.getLeftMotors();
        mRightMotors = mDrivetrain.getRightMotors();

        mLeftEncoder = mLeftLeader.getAlternateEncoder(4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(4096);
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);

        goalDistance = close ? distanceClose : distanceFar;

//        mLeftPIDController = mLeftLeader.getPIDController();
//        mRightPIDController = mRightLeader.getPIDController();

//        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Needs Tuning

        mAlign = new LimelightAlignLeftCommand(mDrivetrain, mVision, true);
    }

    @Override
    public void initialize(){
        stopFlag = false;
        speed = 0.5;
        buffer = 5;
        mVision.updateTargets();
//        goalDistance = goalDistance; //inches
        initDistance = calcDistance();
        goalTravel = initDistance - goalDistance;
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);

        distanceCompleted = false;

        System.out.println("Goal distance (in): "+goalDistance);
        System.out.println("Initial distance (in): "+initDistance);
        System.out.println("Expected travel distance (in) "+goalTravel);

    }

    @Override
    public void execute(){

        actualTravel = ((mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2) * conversion;
//        System.out.println(mRightEncoder.getPosition());
        speed = calcSpeed();
        System.out.println(speed);

        if (mVision.getTargets() >= 1) {
            if (actualTravel < (goalTravel - buffer)){
                mLeftMotors.set(speed);
                mRightMotors.set(speed);
            } else if(actualTravel > (goalTravel + buffer)){
                mLeftMotors.set(-speed);
                mRightMotors.set(-speed);
            } else {
                mLeftMotors.set(0);
                mRightMotors.set(0);
                if (!distanceCompleted) {
                    distanceCompleted = true;
                    System.out.println("Distance Traveled: " + actualTravel);
                }
                stopFlag = true;
            }
        } else {
//            mAlign.findTarget();
        }


//        System.out.println(initDistance);
//        System.out.println(goalTravel - actualTravel);
    }

    @Override
    public boolean isFinished(){
        return stopFlag;
    }

    public double calcDistance(){
        yOffset = mVision.getyOffset();
        return (Constants.LimelightVision.targetHeight - Constants.LimelightVision.cameraHeight)
                / Math.tan(Units.degreesToRadians(Constants.LimelightVision.cameraAngle + yOffset));
    }

    public double calcSpeed(){
        actualTravel = ((mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2) * conversion;
        double calcSpeed = ((goalTravel - actualTravel) / goalTravel);
        double speed = calcSpeed > 1 ? 1 : calcSpeed;
        double finalSpeed = speed < 0.5 ? 0.5 : calcSpeed;
        return finalSpeed;
    }



    public double lowBand(double targetDistance){
        return targetDistance - 24;
    }

    public double highBand(double targetDistance){
        return targetDistance + 24;
    }



}
