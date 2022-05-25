// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autons.AutonRoutine;
import frc.robot.hardcodeAutons.HardCodeAuton;
import frc.robot.hardcodeAutons.TaxiOneBallHardCode;
import frc.robot.commands.*;
import frc.robot.io.Axis;
import frc.robot.limelightvision.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.LimelightVision.*;

import static frc.robot.Constants.driverController;
import static frc.robot.Constants.operatorController;
import static frc.robot.io.Button.*;

public class RobotContainer {

  /***
   * This class is responsible for creating objects for each subsystem and then using method references to bind each function
   * to a button. The auton routine is also established in this class.
   */

  //Subsystems
  private final VPLimelight mLimelightVision = new VPLimelight();
  private final Drivetrain mDrivetrain = new Drivetrain(mLimelightVision);
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Indexer mIndexer = new Indexer();
  private final Climber mClimber = new Climber();

  private final SubsystemBase[] mSubsystems =  {mDrivetrain, mIntake, mShooter, mIndexer, mLimelightVision};

  //Limelight Vision
  private final LimelightAlignCommand mLeftAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.LEFT, TurnMode.TELEOP);
  private final LimelightAlignCommand mRightAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.TELEOP);

  private final LimelightDistanceCommand mDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision, true);

  //Complex Commands (that can't be inlined)
  private final IntakeCargo mIntakeCargo = new IntakeCargo(mIntake);
  private final ClimberControl mClimberControl = new ClimberControl(mClimber);



  //Auton Routines
  private final AutonRoutine ls2bd = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_TWO_BALL_DEFAULT);
  private final AutonRoutine ls2br = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_TWO_BALL_ROLL);
  private final AutonRoutine ls4bd = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_FOUR_BALL_DEFAULT);
  private final AutonRoutine rs2bc = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_TWO_BALL_CLOSE);
  private final AutonRoutine rs2bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_TWO_BALL_FAR);
  private final AutonRoutine rs3bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_THREE_BALL_FAR);
  private final AutonRoutine rs4bc = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FOUR_BALL_CLOSE);
  private final AutonRoutine rs4bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FOUR_BALL_FAR);
  private final AutonRoutine rs5bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FIVE_BALL_FAR);

  //Sendable Chooser object for sending options to Smart Dashboard
  private final SendableChooser<AutonRoutine> mAutons = new SendableChooser<>();


  //Hardcode Autons
  private final HardCodeAuton mAuton = new HardCodeAuton(mDrivetrain, mIntake, mIndexer, mShooter);
  private final TaxiOneBallHardCode mOneBallAuton = new TaxiOneBallHardCode(mDrivetrain, mIntake, mIndexer, mShooter);

  public RobotContainer() {

    //Adding autons to SendableChooser with default option
    mAutons.setDefaultOption(ls2bd.getName(), ls2bd);
    mAutons.addOption(ls2br.getName(), ls2br);
    mAutons.addOption(ls4bd.getName(), ls4bd);
    mAutons.addOption(rs2bc.getName(), rs2bc);
    mAutons.addOption(rs4bc.getName(), rs4bc);
    mAutons.addOption(rs2bf.getName(), rs2bf);
    mAutons.addOption(rs3bf.getName(), rs3bf);
    mAutons.addOption(rs4bf.getName(), rs4bf);
    mAutons.addOption(rs5bf.getName(), rs5bf);

    SmartDashboard.putData(mAutons);

    configureButtonBindings();
    //Sets the state of the MasterDrive method in the Drivetrain class so it can be driven by default
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;

    //Commands that always run unless interrupted by other commands utilizing the subsystem
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::masterDrive, mDrivetrain));
    mIntake.setDefaultCommand(new RunCommand(mIntake::triggerRollerIntake, mIntake));

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
     * Y - Adjust Distance (WIP)
     * X - Align Left
     * B - Align Right
     * A - Toggle Four-Bar
     *
     * Operator Controller
     * Left Bumper - Shoot Close
     * Right Bumper - Shoot Far
     * Y - Feed Indexer
     * X - Shoot Low
     * B - Change Climber Angle
     * A - Enables winch (Hold)
     * Left Y - Raw Winch Control (Requires A to be held)
     *
     */

    //Limelight

    new JoystickButton(driverController, X.getID())
            .whenHeld(mLeftAlign);

    new JoystickButton(driverController, B.getID())
            .whenHeld(mRightAlign);

    new JoystickButton(driverController, Y.getID())
            .whenHeld(mDistanceTarget);

    //Drivetrain

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command

    /*
    new JoystickButton(driverController, LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));
     */

    new JoystickButton(driverController, LEFT_BUMPER.getID())
            .whenPressed(new InstantCommand(mDrivetrain::toggleShifter));

    new JoystickButton(driverController, START.getID())
            .whenPressed(new InstantCommand(mDrivetrain::resetEncoders));

//    Intake

    new JoystickButton(driverController, A.getID())
            .whenPressed(new InstantCommand(mIntake::toggleFourBar));

    new JoystickButton(driverController, RIGHT_BUMPER.getID())
            .whenHeld(mIntakeCargo);

    if (new Joystick(Axis.LEFT_TRIGGER.getID()).getTriggerPressed())
            new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop);

    if (new Joystick(Axis.RIGHT_TRIGGER.getID()).getTriggerPressed())
            new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop);

//    Shooter

    new JoystickButton(operatorController, Y.getID())
            .whenHeld(new StartEndCommand(mIndexer::feedIndexer, mIndexer::setIndexerIdle));

    new JoystickButton(operatorController, RIGHT_BUMPER.getID())
            .whenHeld(new RunCommand(mShooter::shootFar));

    new JoystickButton(operatorController, RIGHT_BUMPER.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

    new JoystickButton(operatorController, LEFT_BUMPER.getID())
            .whenHeld(new RunCommand(mShooter::shootClose));

    new JoystickButton(operatorController, LEFT_BUMPER.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

    new JoystickButton(operatorController, X.getID())
            .whenHeld(new RunCommand(mShooter::rpmShootClose));

    new JoystickButton(operatorController, X.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

//    Climber

    new JoystickButton(operatorController, A.getID())
            .whenHeld(mClimberControl);

    new JoystickButton(operatorController, B.getID())
            .whenPressed(mClimber::angleClimber);

  }

  //Global auton execution called here
  public Command getAutonomousCommand() {

    //Gets the selected auton from the SendableChooser
    AutonRoutine selectedRoutine = mAutons.getSelected();
    selectedRoutine.routineInitialize();

    //Sets the drivetrain up to run the tankDriveVolts method used during auton
    mDrivetrain.mState = Constants.DriveTrain.DriveState.AUTO_DRIVE;

    //Ensures the drivetrain is in high gear to start auton
    mDrivetrain.highGear();

    //The Command object returned is the auton that runs during code execution
    return selectedRoutine.getRoutine();
  }

  //Used in the Robot.java class to ensure the Drivetrain class is running the teleop arcade drive method once the auton ends
  public void activateTeleop() {
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
  }

}
