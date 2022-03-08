package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.util.ArrayList;

import static frc.robot.Constants.Auton.*;


public class AutonGenerator {

    Drivetrain mDrivetrain = new Drivetrain(new VPLimelight()); //hmm
    RamseteController mRamseteController = new RamseteController(Constants.Auton.ramseteB, Constants.Auton.ramseteZeta);
    SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(ks, kv, ka);
    PIDController leftPID = new PIDController(Constants.Auton.kP, 0, 0);
    PIDController rightPID = new PIDController(Constants.Auton.kP, 0, 0);
    Trajectory trajectory;

    //Fills an array with trajectories based on a String array of the paths and returns it
    public ArrayList<Trajectory> getTrajectory(String[] pathing){
        ArrayList<Trajectory> trajectories = new ArrayList<>();
        for (int i = 0; i < pathing.length; i++){
            try {
                trajectories.add(TrajectoryUtil.fromPathweaverJson(
                        Filesystem.getDeployDirectory().toPath().resolve("output/" + pathing[i] + ".wpilib.json")));
            } catch (IOException e){
                System.out.println("Couldn't find Trajectory Path");
                e.printStackTrace();
                return null;
            }
        }
        trajectory = trajectories.get(0);
        return trajectories;
    }

    //Fills an array with Ramsete commands based on a String array of paths and returns it
    public ArrayList<RamseteCommand> getAutonCommands(String[] pathing){
        ArrayList<RamseteCommand> commands= new ArrayList<>();
        mRamseteController.setEnabled(true);

        for (int i = 0; i < pathing.length; i++){
            commands.add(
             new RamseteCommand(
                    getTrajectory(pathing).get(i),
                    mDrivetrain::getPose,
                    mRamseteController,
                    mFeedForward,
                    driveKinematics,
                    mDrivetrain::getWheelSpeeds,
                    leftPID, rightPID,
                    (leftVolts, rightVolts) -> {mDrivetrain.tankDriveVolts(leftVolts, rightVolts);},
                    mDrivetrain)
            );
        }
        return commands;
    }

    public Trajectory getFirstTrajectory(){
        return trajectory;
    }
    //TODO: Create a method that concatenates a trajectory based on the order of the parameters (takes a variable number of parameters?)
}
