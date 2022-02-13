// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autons.AutonTestPosition;
import frc.robot.autons.AutonTestVelocity;
import frc.robot.autons.StopAuton;
import frc.robot.commands.*;
import frc.robot.io.Button;
import frc.robot.limelightvision.LimelightAlignTargetCommand;
import frc.robot.limelightvision.LimelightCompleteVisionCommand;
import frc.robot.limelightvision.LimelightDistanceCommand;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.photonvision.PhotonAlignTargetCommand;
import frc.robot.photonvision.VPPhoton;
import frc.robot.subsystems.*;

public class RobotContainer {

  //Subsystems
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Indexer mIndexer = new Indexer();
  private final Climber mClimber = new Climber();

  //Photon Vision
  private final VPPhoton mPhotonVision = new VPPhoton();
  private final PhotonAlignTargetCommand mPhotonAlignTarget = new PhotonAlignTargetCommand(mPhotonVision, mDrivetrain, mShooter);

  //Limelight Vision
  private final VPLimelight mLimelightVision = new VPLimelight();
  private final LimelightAlignTargetCommand mAlignTarget = new LimelightAlignTargetCommand(mDrivetrain, mLimelightVision);
  private final LimelightDistanceCommand mDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
  private final LimelightAlignTargetCommand mSoloAlignTarget = new LimelightAlignTargetCommand(mDrivetrain, mLimelightVision);
  private final LimelightDistanceCommand mSoloDistanceTarget = new LimelightDistanceCommand(mDrivetrain, mLimelightVision);
  private final LimelightCompleteVisionCommand mCompleteVision = new LimelightCompleteVisionCommand(mAlignTarget, mDistanceTarget);

  //Complex Commands (that can't be inlined)
  private final ShootFar mShootFar = new ShootFar(mShooter, mIndexer);
  private final ShootClose mShootClose = new ShootClose(mShooter, mIndexer);

  //Autons
  private final StopAuton mStopAuton = new StopAuton(mDrivetrain);
  private final AutonTestVelocity mVelocityAuton = new AutonTestVelocity(mDrivetrain);
  private final AutonTestPosition mPositionAuton = new AutonTestPosition(mDrivetrain);

  public RobotContainer() {

    configureButtonBindings();

    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
//    mLimelightVision.setDefaultCommand(new RunCommand(mLimelightVision::printNetworkTables, mLimelightVision));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(Constants.driverController, Button.ButtonID.X.getID())
            .whenHeld(mSoloAlignTarget);

    new JoystickButton(Constants.driverController, Button.ButtonID.B.getID())
            .whenHeld(mSoloDistanceTarget);

    new JoystickButton(Constants.driverController, Button.ButtonID.A.getID())
            .whenHeld(mCompleteVision);

    //If the left bumper is pressed and the drivetrain is in low gear perform the first command
    //If the left bumper is pressed and the drivetrain is in high gear perform the second command
    new JoystickButton(Constants.driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));
/*
    Waiting for Build

    new JoystickButton(Constants.driverController, Button.ButtonID.RIGHT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mIntake::retractIntake),
                    new InstantCommand(mIntake::extendIntake),
                    mIntake::getFourBarState));

    new JoystickButton(Constants.driverController, Button.ButtonID.A.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerIntake, mIntake::rollerStop));

    new JoystickButton(Constants.driverController, Button.ButtonID.B.getID())
            .whenHeld(new StartEndCommand(mIntake::rollerOuttake, mIntake::rollerStop));

    new JoystickButton(Constants.driverController, Button.ButtonID.X.getID())
            .whenHeld(mShootFar);

    new JoystickButton(Constants.driverController, Button.ButtonID.X.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(Constants.driverController, Button.ButtonID.X.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
            .whenHeld(mShootClose);

    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
            .whenInactive(mShooter::setShooterIdle);

    new JoystickButton(Constants.driverController, Button.ButtonID.Y.getID())
            .whenInactive(mIndexer::setIndexerIdle);

    new JoystickButton(Constants.operatorController, Button.ButtonID.A.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mClimber::angleA),
                    new InstantCommand(mClimber::angleB),
                    mClimber::getAngle));

 */

  }

  public Command getAutonomousCommand() {

    return mVelocityAuton;
//    return null;
  }

}
