package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import static frc.robot.Constants.Auton.*;


public class AutonGenerator {

    Drivetrain mDrivetrain;
    RamseteController mRamseteController = new RamseteController(Constants.Auton.ramseteB, Constants.Auton.ramseteZeta);
    SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(ks, kv, ka);
    PIDController leftPID = new PIDController(Constants.Auton.kP, 0, 0);
    PIDController rightPID = new PIDController(Constants.Auton.kP, 0, 0);

    //Fills an array with trajectories based on a String array of the paths and returns it
    public Trajectory getTrajectory(String pathing){
//        ArrayList<Trajectory> trajectories = new ArrayList<>();
        Trajectory trajectory;
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("output/" + pathing + ".wpilib.json");
            trajectory = (TrajectoryUtil.fromPathweaverJson(path));
            System.out.println("Added trajectory " + " to array");
        } catch (IOException e){
            System.out.println("Couldn't find trajectory path");
            e.printStackTrace();
            return null;
        }

//        trajectory = trajectories.get(0);
        if (trajectory != null){
            System.out.println("Retrieved first trajectory");
        }
        return trajectory;
    }

    //Fills an array with Ramsete commands based on a String array of paths and returns it
    public ArrayList<RamseteCommand> getAutonCommands(String[] pathing, Drivetrain subsystem){
        ArrayList<RamseteCommand> commands= new ArrayList<>();
        mRamseteController.setEnabled(true);
        mDrivetrain = subsystem;

        for (int i = 0; i < pathing.length; i++){
            System.out.println("Adding trajectory " + i + " ...Hopefully");
            Trajectory trajectory = getTrajectory(pathing[i]);
            if (trajectory == null){
                System.out.println("Trajectory " + i + " not found");
                return null;
            }
            commands.add(
                    new RamseteCommand(
                            trajectory,
                            mDrivetrain::getPose,
                            mRamseteController,
                            mFeedForward,
                            driveKinematics,
                            mDrivetrain::getWheelSpeeds,
                            leftPID, rightPID,
                            (leftVolts, rightVolts) -> {mDrivetrain.tankDriveVolts(leftVolts, rightVolts);},
                            mDrivetrain));
        }
        return commands;
    }
}
