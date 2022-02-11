package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;
import frc.robot.utility.PIDConfig;

public class CorrectDistanceLimelight extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private AlignTargetLimeLight mAlign;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private double yOffset, speed, goalDistance, initDistance, goalTravel, actualTravel, buffer;

    private final double conversion = 0.1429 * 6 * Math.PI * 2; //Rotations to inches

    private boolean distanceCompleted;

    public CorrectDistanceLimelight(Drivetrain subsystemA, VPLimelight subsystemB){
        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftEncoder = mLeftLeader.getAlternateEncoder(4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(4096);
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);

        mLeftPIDController = mLeftLeader.getPIDController();
        mRightPIDController = mRightLeader.getPIDController();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Needs Tuning

        mAlign = new AlignTargetLimeLight(mDrivetrain, mVision);
    }

    @Override
    public void initialize(){
        speed = 0.25;
        buffer = 10;
        mVision.updateTargets();
        goalDistance = 120; //inches
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


        if (mVision.getTargets() >= 1) {
            if (actualTravel < (goalTravel - buffer)){
                mLeftLeader.set(-speed);
                mRightLeader.set(-speed);
            } else if(actualTravel > (goalTravel + buffer)){
                mLeftLeader.set(speed);
                mRightLeader.set(speed);
            } else {
                mLeftLeader.set(0);
                mRightLeader.set(0);
                if (!distanceCompleted) {
                    distanceCompleted = true;
                    System.out.println("Distance Traveled: " + actualTravel);
                }
            }
        } else {
//            mAlign.findTarget();
        }


//        System.out.println(initDistance);
//        System.out.println(goalTravel - actualTravel);
    }

    public double calcDistance(){
        yOffset = mVision.getyOffset();
        return (Constants.LimelightVision.targetHeight - Constants.LimelightVision.cameraHeight)
                / Math.tan(Units.degreesToRadians(Constants.LimelightVision.cameraAngle + yOffset));
    }

    public void adjustDistance(){


    }

    public double lowBand(double targetDistance){
        return targetDistance - 24;
    }

    public double highBand(double targetDistance){
        return targetDistance + 24;
    }

}
