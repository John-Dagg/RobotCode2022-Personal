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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autons.AutonGenerator;
import frc.robot.commands.*;
import frc.robot.io.Axis;
import frc.robot.io.Button;
import frc.robot.limelightvision.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.LimelightVision.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import static frc.robot.Constants.driverController;
import static frc.robot.Constants.operatorController;

public class RobotContainer {

  private final VPLimelight mLimelightVision = new VPLimelight();
  //Subsystems
  private final Drivetrain mDrivetrain = new Drivetrain(mLimelightVision);
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Indexer mIndexer = new Indexer();
  private final Climber mClimber = new Climber();

  //Limelight Vision

  private final LimelightAlignCommand mLeftAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.LEFT, TurnMode.TELEOP);
  private final LimelightAlignCommand mRightAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.TELEOP);
  private final LimelightDistanceCommand mDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision, true);
//  private final LimelightAlignLeftCommand mLeftAlign = new LimelightAlignLeftCommand(mDrivetrain, mLimelightVision);
//  private final LimelightAlignRightCommand mRightAlign = new LimelightAlignRightCommand(mDrivetrain, mLimelightVision);
//  private final LimelightDistanceCommand mSoloDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
//  private final LimelightDistanceCommand mDistance = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
//  private final LimelightCompleteVisionCommand mCompleteVision = new LimelightCompleteVisionCommand(mAlignTarget, mDistanceTarget);

  //Complex Commands (that can't be inlined)
  private final ShootFar mShootFar = new ShootFar(mShooter, mIndexer);
  private final ShootClose mShootClose = new ShootClose(mShooter, mIndexer);
  private final IntakeCargo mIntakeCargo = new IntakeCargo(mIntake);

  //Auton
  private AutonGenerator autonGenerator = new AutonGenerator();
  private ArrayList<RamseteCommand> ramseteCommands;
  private String[] mFiveBallAuton = {"FiveBallA", "FiveBallB", "FiveBallC", "FiveBallD"};

  public RobotContainer() {

    configureButtonBindings();
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE;
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
//    mIntake.setDefaultCommand(new RunCommand(mIntake::triggerRollerIntake, mIntake));
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
     * Left X and Right Y - Arcade Drive                TODO: Fix stuttering -> "Differential Drive not updated often enough" /Brownout?
     * Left Bumper - Shifts Gears (Toggle)
     * Right Bumper - Extend/Retract Intake (Toggle)    TODO: Toggle or Hold?
     * Right Trigger - Intake                           TODO: Get build to add Bumpers so it works / Test
     * Left Trigger - Outtake                           TODO: ^^^
     * X - Align Target Angle + Default Turn Left       TODO: Tune - Why is the pipeline unstable?
     * B - Align Target Angle + Default Turn Right      TODO: Tune ^
     * Y - Align Distance Close (Disabled)              TODO: Find appropriate distance and tune calcDistance() {@link LimelightDistanceCommand}
     * A - Align Distance Far   (Disabled)              TODO: ^^^
     *
     * Operator Controller
     * X - Shoot Far                                    Sequential Command Group with subsystem dependencies 1st Try!
     * Y - Shoot Close
     * A - Change Climber Angle (Disabled)
     * B - Engage Brake                                 TODO: Change climb to better binds
     * Left Bumper - Winch Up                           TODO: Test and change to joystick + hold button
     * Right Bumper - Winch Down                        TODO: ^^^
     *
     * Auton                                            TODO: AutonGenerator is returning a nullPointer? Not filling array correctly?
     *                                                        Tune PID constants and ka, kv, ks
     *                                                        Check encoders
     *                                                        Add timer to shoot close/far commands so they actually end in auton
     *                                                        Test Sequential Command Groups and Parallel Race Groups in auton
     */


//    new POVButton(Constants.driverController, 90)
//            .whenHeld(mRightAlign);
//    new POVButton(Constants.driverController, 270)
//            .whenHeld(mLeftAlign);

//    new JoystickButton(driverController, Button.ButtonID.X.getID())
//            .whenHeld(mLeftAlign);

//    new JoystickButton(driverController, Button.ButtonID.B.getID())
//            .whenHeld(mRightAlign);

//    new JoystickButton(driverController, Button.ButtonID.Y.getID())
//            .whenHeld(mDistanceTarget);

//    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
//            .whenHeld(mAlignTarget.andThen(new WaitCommand(1)).andThen(mDistance));

    //Drivetrain

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command
    new JoystickButton(driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));

//    Waiting for Build lol
//    Intake

    new JoystickButton(driverController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mIntake::retractIntake),
                    new InstantCommand(mIntake::extendIntake),
                    mIntake::getFourBarState));

    //  Another option for intake control to add more buttons hopefully

//    if (new Joystick(Axis.AxisID.LEFT_TRIGGER.getID()).getTriggerPressed())
//            new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop);

//    if (new Joystick(Axis.AxisID.RIGHT_TRIGGER.getID()).getTriggerPressed())
//            new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop);

    new JoystickButton(driverController, Button.ButtonID.A.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop));

    new JoystickButton(driverController, Button.ButtonID.B.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop));

//    Shooter

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenHeld(mShootFar);

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenHeld(mShootClose);

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mIndexer::setIndexerIdle);

//    Climber


//    new JoystickButton(Constants.operatorController, Button.ButtonID.A.getID())
//            .whenPressed(new ConditionalCommand(
//                    new InstantCommand(mClimber::angleA),
//                    new InstantCommand(mClimber::angleB),
//                    mClimber::getAngle));

    new JoystickButton(operatorController, Button.ButtonID.B.getID())
            .whenPressed(new InstantCommand(mClimber::brake));

    new JoystickButton(operatorController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenHeld(new StartEndCommand(mClimber::winchUp, mClimber::winchStop));

    new JoystickButton(operatorController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenHeld(new StartEndCommand(mClimber::winchDown, mClimber::winchStop));



  }

  //Global auton execution called here
  public Command getAutonomousCommand() {

    ramseteCommands = autonGenerator.getAutonCommands(mFiveBallAuton, mDrivetrain);

    mDrivetrain.resetOdometry(autonGenerator.getFirstTrajectory().getInitialPose());

    //TODO: Find a better way to do this
    /*
    SequentialCommandGroup auton = mLeftAlign.andThen(mShootClose).andThen(ramseteCommands.get(0))
            .andThen(ramseteCommands.get(1)).andThen(mLeftAlign).andThen(mShootClose)
            .andThen(ramseteCommands.get(2)).andThen(ramseteCommands.get(3)).andThen(mDistanceTarget)
            .andThen(mShootFar).andThen(mDrivetrain::stopDrive);

     */
    /***
    * Aim -> Shoot -> Drive and Collect -> Aim -> Shoot -> Drive and Collect -> Correct Distance -> Aim -> Shoot -> Stop
    */

    //TODO: Shooter won't stop I think
    SequentialCommandGroup auton = new SequentialCommandGroup(
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootClose(mShooter, mIndexer, 2),
            new ParallelRaceGroup(ramseteCommands.get(0).andThen(ramseteCommands.get(1)), new IntakeCargo(mIntake)),
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootClose(mShooter, mIndexer, 4),
            new ParallelRaceGroup(ramseteCommands.get(2), new IntakeCargo(mIntake)),
            ramseteCommands.get(3),
            new LimelightDistanceCommand(mDrivetrain, mLimelightVision, false),
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootFar(mShooter, mIndexer, 4),
            new InstantCommand(mDrivetrain::stopDrive));

  return ramseteCommands.get(0).andThen(new InstantCommand(mDrivetrain::stopDrive));
//  return auton;
//  return null;
  }

}
