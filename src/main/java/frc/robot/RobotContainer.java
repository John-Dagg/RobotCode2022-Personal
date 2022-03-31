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
import frc.robot.autons.HardCodeAuton;
import frc.robot.autons.TaxiOneBallHardCode;
import frc.robot.commands.*;
import frc.robot.io.Axis;
import frc.robot.io.Button;
import frc.robot.limelightvision.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.LimelightVision.*;

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

  private final SubsystemBase[] mSubsystems =  {mDrivetrain, mIntake, mShooter, mIndexer, mLimelightVision};

  //Limelight Vision
  private final LimelightAlignCommand mLeftAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.LEFT, TurnMode.TELEOP);
  private final LimelightAlignCommand mRightAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.TELEOP);
  private final LimelightAlignCommand mAutonLeftAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.LEFT, TurnMode.AUTON);
  private final LimelightAlignCommand mAutonRightAlign = new LimelightAlignCommand(mDrivetrain, mLimelightVision, TurnDirection.RIGHT, TurnMode.AUTON);

  private final LimelightDistanceCommand mDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision, true);

  //Complex Commands (that can't be inlined)
  private final ShootFar mShootFar = new ShootFar(mShooter, mIndexer);
  private final ShootClose mShootClose = new ShootClose(mShooter, mIndexer);
  private final ShootLow mShootLow = new ShootLow(mShooter, mIndexer, Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE);
  private final ShootLow mAutoLow = new ShootLow(mShooter, mIndexer, Constants.DriveTrain.DriveState.AUTO_DRIVE);
  private final IntakeCargo mIntakeCargo = new IntakeCargo(mIntake);
  private final ClimberControl mClimberControl = new ClimberControl(mClimber);
  private final ShootCustom mCustomShooter = new ShootCustom(mShooter, mIndexer, mLimelightVision);



  //Auton Routines
//  private final AutonRoutine mThreeBall = new AutonRoutine(mSubsystems, AutonRoutine.Routine.THREE_BALL_TEST);
//  private final AutonRoutine mFiveBall = new AutonRoutine(mSubsystems, AutonRoutine.Routine.FIVE_BALL_TEST);
//

  private final AutonRoutine ls2bd = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_TWO_BALL_DEFAULT);
  private final AutonRoutine ls2br = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_TWO_BALL_ROLL);
  private final AutonRoutine ls4bd = new AutonRoutine(mSubsystems, AutonRoutine.Routine.LEFT_SIDE_FOUR_BALL_DEFAULT);
  private final AutonRoutine rs2bc = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_TWO_BALL_CLOSE);
  private final AutonRoutine rs2bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_TWO_BALL_FAR);
  private final AutonRoutine rs3bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_THREE_BALL_FAR);
  private final AutonRoutine rs4bc = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FOUR_BALL_CLOSE);
  private final AutonRoutine rs4bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FOUR_BALL_FAR);
  private final AutonRoutine rs5bf = new AutonRoutine(mSubsystems, AutonRoutine.Routine.RIGHT_SIDE_FIVE_BALL_FAR);

  private final SendableChooser<AutonRoutine> mAutons = new SendableChooser<>();


  //Hardcode
  private final HardCodeAuton mAuton = new HardCodeAuton(mDrivetrain, mIntake, mIndexer, mShooter);
  private final TaxiOneBallHardCode mOneBallAuton = new TaxiOneBallHardCode(mDrivetrain, mIntake, mIndexer, mShooter);

  public RobotContainer() {

//    mAutons.setDefaultOption(mThreeBall.getName(), mThreeBall);
//    mAutons.addOption(mFiveBall.getName(), mFiveBall);
//
    mAutons.setDefaultOption(ls2bd.getName(), ls2bd);
    mAutons.addOption(ls2br.getName(), ls2br);
    mAutons.addOption(ls4bd.getName(), ls4bd);
    mAutons.addOption(rs2bc.getName(), rs2bc);
    mAutons.addOption(rs2bf.getName(), rs2bf);
    mAutons.addOption(rs3bf.getName(), rs3bf);
    mAutons.addOption(rs4bc.getName(), rs4bc);
    mAutons.addOption(rs4bf.getName(), rs4bf);
    mAutons.addOption(rs5bf.getName(), rs5bf);

    SmartDashboard.putData(mAutons);

    configureButtonBindings();
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
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
     * X - Shooter Arcade Drive Toggle
     * B - Intake Arcade Drive Toggle
     * A - Toggle Four-Bar
     *
     * Operator Controller
     * Left Bumper - Shoot Far
     * Right Bumper - Shoot Close
     * Y - Shoot Low (In Theory)
     * A - Disengages brake + enables winch (Hold)
     *
     * B - Toggle Brake
     * Left Y - Raw Winch Control (Requires A to be held)
     *
     * Auton
     *
     *     (Disabled)
     * X - Align Target Angle + Default Turn Left
     * B - Align Target Angle + Default Turn Right
     * B - Change Climber Angle
     *
     */

    //Limelight

    new JoystickButton(driverController, Button.ButtonID.X.getID())
            .whenHeld(mLeftAlign);

    new JoystickButton(driverController, Button.ButtonID.B.getID())
            .whenHeld(mRightAlign);

    new JoystickButton(driverController, Button.ButtonID.Y.getID())
            .whenHeld(mDistanceTarget);

    //Drivetrain

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command

    new JoystickButton(driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));

    new JoystickButton(driverController, Button.ButtonID.START.getID())
            .whenPressed(new InstantCommand(mDrivetrain::resetEncoders));

//    new JoystickButton(driverController, Button.ButtonID.SELECT.getID())
//            .whenHeld(new StartEndCommand(mLimelightVision::steadyArray, mLimelightVision::offArray));


//    new JoystickButton(driverController, Button.ButtonID.LEFT_BUMPER.getID())
//            .whenHeld(new StartEndCommand(mDrivetrain::lowGear, mDrivetrain::highGear));

//    new JoystickButton(driverController, Button.ButtonID.START.getID())
//            .whenPressed(new InstantCommand(mDrivetrain::lowGear));

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
            .whenHeld(new StartEndCommand(mIndexer::feedIndexer, mIndexer::setIndexerIdle));

    new JoystickButton(operatorController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenHeld(new RunCommand(mShooter::shootFar));

    new JoystickButton(operatorController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

    new JoystickButton(operatorController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenHeld(new RunCommand(mShooter::shootClose));

    new JoystickButton(operatorController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenHeld(new RunCommand(mShooter::shootLow));

    new JoystickButton(operatorController, Button.ButtonID.X.getID())
            .whenReleased(new RunCommand(mShooter::setShooterIdle));

//    new JoystickButton(operatorController, Button.ButtonID.X.getID())
//            .whenHeld(mShootLow);

//    new POVButton(operatorController, 90)
//            .whenPressed(new InstantCommand(mShooter::setAngle));

//    new JoystickButton(operatorController, Button.ButtonID.START.getID())
//            .whenHeld(mCustomShooter);

//    Climber

    new JoystickButton(operatorController, Button.ButtonID.A.getID())
            .whenHeld(mClimberControl);

    new JoystickButton(operatorController, Button.ButtonID.B.getID())
            .whenPressed(mClimber::angleClimber);

  }

  //Global auton execution called here
  public Command getAutonomousCommand() {

    AutonRoutine selectedRoutine = mAutons.getSelected();
    selectedRoutine.routineInitialize();

    mDrivetrain.mState = Constants.DriveTrain.DriveState.AUTO_DRIVE;
    mDrivetrain.highGear();

    return selectedRoutine.getRoutine();
  }

  public void activateTeleop() {
    mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
  }

}
