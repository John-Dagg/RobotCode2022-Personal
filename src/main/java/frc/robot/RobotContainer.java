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
import frc.robot.autons.HardCodeAuton;
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

  //Subsystems
  private final VPLimelight mLimelightVision = new VPLimelight();
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
  private final ClimberControl mClimberControl = new ClimberControl(mClimber);

  //Auton
  private AutonGenerator autonGenerator = new AutonGenerator();
  private ArrayList<RamseteCommand> ramseteCommands;
  private String[] mFiveBallAuton = {"FiveBallA", "FiveBallB", "FiveBallC", "FiveBallD"};
  private String[] mTest = {"test"};

  //Hardcode
  private final HardCodeAuton mAuton = new HardCodeAuton(mDrivetrain, mIntake, mIndexer, mShooter);

  public RobotContainer() {

    configureButtonBindings();
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::masterDrive, mDrivetrain));
    mIntake.setDefaultCommand(new RunCommand(mIntake::triggerRollerIntake, mIntake));
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
     * Left Bumper - Shifts Gears (Toggle)
     * Right Bumper - Extends + Intakes (Hold)
     * Right Trigger - Intake
     * Left Trigger - Outtake
     * X - Align Target Angle + Default Turn Left       TODO: Tune - Why is the pipeline unstable?
     * B - Align Target Angle + Default Turn Right      TODO: Tune ^
     * Y - Drivetrain Toggle                            TODO: Limelight - Find appropriate distance and tune calcDistance() {@link LimelightDistanceCommand}
     * A - Extend Intake and Spin Rollers               TODO:
     *
     * Operator Controller
     * Y - Shoot Far
     * X - Shoot Close
     * B - Change Climber Angle (Disabled)
     * A - Disengages brake + enables winch (Hold)
     * B - Toggle Brake
     * Left X - Raw Winch Control (Requires A to be held)
     *
     * Auton                                            TODO: AutonGenerator is returning a nullPointer? Not filling array correctly?
     *                                                        Tune PID constants and ka, kv, ks
     *                                                        Check encoders
     *                                                        Add timer to shoot close/far commands so they actually end in auton
     *                                                        Test Sequential Command Groups and Parallel Race Groups in auton
     */

    //Limelight

//    new JoystickButton(driverController, Button.ButtonID.X.getID())
//            .whenHeld(mLeftAlign);
//
//    new JoystickButton(driverController, Button.ButtonID.B.getID())
//            .whenHeld(mRightAlign);

    //Drivetrain

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command
    new JoystickButton(driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));

    new JoystickButton(driverController, Button.ButtonID.Y.getID())
            .whenPressed(new InstantCommand(mDrivetrain::toggleArcadeStyle));




//    Intake

    new JoystickButton(driverController, Button.ButtonID.A.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mIntake::retractIntake),
                    new InstantCommand(mIntake::extendIntake),
                    mIntake::getFourBarState));

    new JoystickButton(driverController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenHeld(mIntakeCargo);

    //  Another option for intake control to add more buttons hopefully
    //  Leaving this here for now. Using default command

    if (new Joystick(Axis.AxisID.LEFT_TRIGGER.getID()).getTriggerPressed())
            new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop);

    if (new Joystick(Axis.AxisID.RIGHT_TRIGGER.getID()).getTriggerPressed())
            new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop);

//    Shooter

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenHeld(mShootFar);

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(operatorController, Button.ButtonID.Y.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenHeld(mShootClose);

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenInactive(mIndexer::setIndexerIdle);

//    Climber

//    new JoystickButton(Constants.operatorController, Button.ButtonID.A.getID())
//            .whenPressed(new ConditionalCommand(
//                    new InstantCommand(mClimber::angleA),
//                    new InstantCommand(mClimber::angleB),
//                    mClimber::getAngle));

    new JoystickButton(operatorController, Button.ButtonID.A.getID())
            .whenHeld(mClimberControl);

    new JoystickButton(operatorController, Button.ButtonID.B.getID())
            .whenPressed(new InstantCommand(mClimber::brake));

    new JoystickButton(operatorController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenHeld(new StartEndCommand(mClimber::winchUp, mClimber::winchStop));

    new JoystickButton(operatorController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenHeld(new StartEndCommand(mClimber::winchDown, mClimber::winchStop));



  }

  //Global auton execution called here
  public Command getAutonomousCommand() {

//    ramseteCommands = autonGenerator.getAutonCommands(mTest, mDrivetrain);

//    mDrivetrain.resetOdometry(autonGenerator.getTrajectory(mTest[0]).getInitialPose());

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
    /*
    SequentialCommandGroup auton = new SequentialCommandGroup(
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootClose(mShooter, mIndexer, 2),
            new ParallelRaceGroup(ramseteCommands.get(0).andThen(ramseteCommands.get(1)), new IntakeCargo(mIntake)),
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootClose(mShooter, mIndexer, 2),
            new ParallelRaceGroup(ramseteCommands.get(2), new IntakeCargo(mIntake)),
            ramseteCommands.get(3),
            new LimelightDistanceCommand(mDrivetrain, mLimelightVision, false),
            new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON),
            new ShootFar(mShooter, mIndexer, 2),
            new InstantCommand(mDrivetrain::stopDrive));

     */

  //Hard Coding ):
  return mAuton;

//  return ramseteCommands.get(0).andThen(new InstantCommand(mDrivetrain::stopDrive));
//  return auton;
//  return null;
  }

}
