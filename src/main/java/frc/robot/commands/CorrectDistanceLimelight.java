package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;
import frc.robot.utility.PIDConfig;

public class CorrectDistanceLimelight extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;

    public CorrectDistanceLimelight(Drivetrain subsystemA, VPLimelight subsystemB){
        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftPIDController = mLeftLeader.getPIDController();
        mRightPIDController = mRightLeader.getPIDController();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Needs Tuning
    }

}
