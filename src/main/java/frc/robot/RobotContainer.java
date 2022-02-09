// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.io.Button;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Indexer mIndexer = new Indexer();
  private final VisionProcessingPhoton mVision = new VisionProcessingPhoton();
  private final VPLimelight mLimelightVision = new VPLimelight();

  private final AlignTargetLimeLight mAlignTarget = new AlignTargetLimeLight(mDrivetrain, mLimelightVision);

  private final ShootFar mShootFar = new ShootFar(mShooter, mIndexer);
  private final ShootClose mShootClose = new ShootClose(mShooter, mIndexer);
  private final StopAuton m_autoCommand = new StopAuton(mDrivetrain);
//  private final AlignTarget mAlignTarget = new AlignTarget(mVision, mDrivetrain, mShooter);
  //  private final AutonTest m_autoCommand = new AutonTest(mDrivetrain);

  public RobotContainer() {

    configureButtonBindings();

    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
//    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::getYaw, mDrivetrain));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(Constants.operatorController, Button.ButtonID.A.getID())
            .whenHeld(mAlignTarget);




    new JoystickButton(Constants.driverController, Button.ButtonID.LEFT_BUMPER.getID())
            .whenPressed(new ConditionalCommand(
                    new InstantCommand(mDrivetrain::highGear),
                    new InstantCommand(mDrivetrain::lowGear),
                    mDrivetrain::getLowGear));

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
/*
    new JoystickButton(Constants.driverController, Button.ButtonID.A.getID())
            .whenHeld(mAlignTarget);
*/
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

//Uncomment code in the Robot class
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

}
