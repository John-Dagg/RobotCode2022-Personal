// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.io.Button;
import frc.robot.limelightvision.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  //Subsystems
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Indexer mIndexer = new Indexer();
//  private final Climber mClimber = new Climber();

  //Limelight Vision
//  private final VPLimelight mLimelightVision = new VPLimelight();
//  private final LimelightAlignLeftCommand mAlignTarget = new LimelightAlignLeftCommand(mDrivetrain, mLimelightVision);
//  private final LimelightDistanceCommand mDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
//  private final LimelightAlignLeftCommand mLeftAlign = new LimelightAlignLeftCommand(mDrivetrain, mLimelightVision);
//  private final LimelightAlignRightCommand mRightAlign = new LimelightAlignRightCommand(mDrivetrain, mLimelightVision);
//  private final LimelightDistanceCommand mSoloDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
//  private final LimelightDistanceCommand mDistance = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
//  private final LimelightCompleteVisionCommand mCompleteVision = new LimelightCompleteVisionCommand(mAlignTarget, mDistanceTarget);

  //Complex Commands (that can't be inlined)
//  private final ShootFar mShootFar = new ShootFar(mShooter, mIndexer);
//  private final ShootClose mShootClose = new ShootClose(mShooter, mIndexer);

  //Auton
  private final String soloPath = "Curve";
  private final String soloTrajectoryFile = "output/"+soloPath+".wpilib.json";

  private final String testPath = "test1";
  private final String testTrajectoryFile = "output/"+soloPath+".wpilib.json";

  private Path soloTrajectoryPath;
  private Trajectory soloTrajectory;

  private String pathing1 = "5BallPath1"; //Change this to change trajectory
  private String pathing2 = "5BallPath2";
  private String pathing3 = "5BallPath3";

  private final String pathingTest = "test1";


  private String trajectoryFile1 = "output/"+pathing1+".wpilib.json";
  private String trajectoryFile2 = "output/"+pathing2+".wpilib.json";
  private String trajectoryFile3 = "output/"+pathing3+".wpilib.json";

  private final String trajectoryFileTest = "output/"+pathingTest+".wpilib.json";

  private Path trajectoryPath1;
  private Path trajectoryPath2;
  private Path trajectoryPath3;
  private Path trajectoryPathTest;
  private Trajectory trajectory1;
  private Trajectory trajectory2;
  private Trajectory trajectory3;
  private Trajectory trajectoryTest;

  private Trajectory fullTrajectory;
  private RamseteCommand autonCommandA;
//  private LimelightAlignRightCommand autonCommandB = new LimelightAlignRightCommand(mDrivetrain, mLimelightVision);
//  private ShootClose autonCommandC = new ShootClose(mShooter, mIndexer);

  public RobotContainer() {

    configureButtonBindings();

    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
//    mIndexer.setDefaultCommand(new RunCommand(mIndexer::indexerTest, mIndexer));
//    mShooter.setDefaultCommand(new RunCommand(mShooter::shooterTest, mShooter));
//    mLimelightVision.setDefaultCommand(new RunCommand(mLimelightVision::printNetworkTables, mLimelightVision));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /***
     * Driver Controller
     * Left X and Right Y - Arcade Drive
     * Left Bumper - Shifts Gears
     * Right Bumper - Extend/Retract Intake
     * A - Intake
     * B - Outtake
     * X - Align Target Angle
     * Y - Align Target Distance
     *
     * Operator Controller
     * X - Shoot Far
     * Y - Shoot Close
     * A - Change Climber Angle
     * B - Engage Brake
     * Left Bumper - Winch Up
     * Right Bumper - Winch Down
     */

    /*
    new JoystickButton(Constants.driverController, Button.ButtonID.X.getID())
            .whenHeld(mLeftAlign);

//    new JoystickButton(Constants.driverController, Button.ButtonID.B.getID())
//            .whenHeld(mRightAlign);

    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
            .whenHeld(mSoloDistanceTarget);

//    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
//            .whenHeld(mAlignTarget.andThen(new WaitCommand(1)).andThen(mDistance));

     */

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command
    new JoystickButton(Constants.driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));
/*
    Waiting for Build lol

    new JoystickButton(Constants.driverController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mIntake::retractIntake),
                    new InstantCommand(mIntake::extendIntake),
                    mIntake::getFourBarState));

    new JoystickButton(Constants.driverController, Button.ButtonID.A.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop));

    new JoystickButton(Constants.driverController, Button.ButtonID.B.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop));

    new JoystickButton(Constants.operatorController, Button.ButtonID.X.getID())
            .whenHeld(mShootFar);

    new JoystickButton(Constants.operatorController, Button.ButtonID.X.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(Constants.operatorController, Button.ButtonID.X.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(Constants.operatorController, Button.ButtonID.Y.getID())
            .whenHeld(mShootClose);

    new JoystickButton(Constants.operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(Constants.operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(Constants.operatorController, Button.ButtonID.A.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mClimber::angleA),
                    new InstantCommand(mClimber::angleB),
                    mClimber::getAngle));

    new JoystickButton(Constants.operatorController, Button.ButtonID.B.getID())
            .whenPressed(new InstantCommand(mClimber::brake);

    new JoystickButton(Constants.operatorController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenHeld(new RunCommand(mClimber::winchUp));

    new JoystickButton(Constants.operatorController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenHeld(new RunCommand(mClimber::winchDown));

 */

  }

  //Global auton execution called here
  public Command getAutonomousCommand() {
/*
    try {

      trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);

      trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);

      trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile3);
      trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);

      soloTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(soloTrajectoryFile);
      soloTrajectory = TrajectoryUtil.fromPathweaverJson(soloTrajectoryPath);

      trajectoryPathTest = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileTest);
      trajectoryTest = TrajectoryUtil.fromPathweaverJson(trajectoryPathTest);



    } catch (IOException e){
      System.out.println("Couldn't find trajectory path");
      e.printStackTrace();
    }
    // How to concatenate trajectories (currently not in use)
    fullTrajectory = trajectory1.concatenate(trajectory2).concatenate(trajectory3);

    RamseteController mController = new RamseteController(Constants.Auton.ramseteB, Constants.Auton.ramseteZeta);

    // Set false for testing purposes ONLY
    mController.setEnabled(true);

    PIDController leftPID = new PIDController(Constants.Auton.kP, 0, 0);
    PIDController rightPID = new PIDController(Constants.Auton.kP, 0, 0);

    autonCommandA = new RamseteCommand(
            trajectoryTest,
            mDrivetrain::getPose,
            mController,
            new SimpleMotorFeedforward(
                    Constants.Auton.ks,
                    Constants.Auton.kv,
                    Constants.Auton.ka),
            Constants.Auton.driveKinematics,
            mDrivetrain::getWheelSpeeds,
            leftPID,
            rightPID,
            (leftVolts, rightVolts) -> {
              mDrivetrain.tankDriveVolts(leftVolts, rightVolts); }, mDrivetrain);

    mDrivetrain.resetOdometry(soloTrajectory.getInitialPose());

//    return autonCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));
    return new SequentialCommandGroup(autonCommandA.andThen(() -> mDrivetrain.tankDriveVolts(0,0)),
                                      autonCommandB,
                                      autonCommandC);

 */
  return null;
  }

}
