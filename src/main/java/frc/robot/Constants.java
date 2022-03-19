// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final Joystick driverController = new Joystick(0);
    public static final Joystick operatorController = new Joystick(1);

    public static final class Auton {

        public static final double ks = 0.19409; //Volts
        public static final double kv = 1.8784; //Volt seconds per meter
        public static final double ka = 0.10279 ; //Volt seconds squared per meter

        public static final double kP = 3.4; //Proportional Gain

        public static final double robotWidth = Units.inchesToMeters(24.0);
        public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(robotWidth);

        //Constants for using the ramsete controller
        public static final double ramseteB = 2;
        public static final double ramseteZeta = 0.7;
    }

    public static final class DriveTrain {

        public static final int leftLeaderPort = 1;
        public static final int leftFollowerAPort = 2;
        public static final int leftFollowerBPort = 3;
        public static final int rightLeaderPort = 5;
        public static final int rightFollowerAPort = 6;
        public static final int rightFollowerBPort = 7;

        public static final int[] shifterPorts = {13, 4, 5};

        public static final double deadband = 0.05;

        public enum DriveState { TELE_DRIVE_INTAKE, TELE_DRIVE_SHOOTER, TELE_LIMELIGHT, LIMELIGHT_DRIVE, AUTO_DRIVE, AUTO_LIMELIGHT; }

        public static final DriveState defaultState = DriveState.TELE_DRIVE_INTAKE;

    }

    public static final class Intake {

        public static final int intakeMotorPort = 4;

        public static final int[] fourBarPorts = {0, 6, 7};
    }

    public static final class Indexer {

        public static final int indexerPort = 8;
    }

    public static final class Shooter {

        public static final int shooterAPort = 9;
        public static final int shooterBPort = 10;

        public static final int[] anglerPorts = {13, 6, 7};
    }

    public static final class Climber{

        public static final int climberPortA = 11;
        public static final int climberPortB = 12;
        public static final int[] solenoidPorts = {13, 0, 1};
        public static final int[] brakePorts = {13, 2, 3};

        public static final double deadband = 0.1;
    }

    public static final class PhotonVision{

        public static final double cameraHeight = Units.inchesToMeters(25);
        public static final double cameraPitch = Units.degreesToRadians(0);
        public static final double targetHeightA = Units.inchesToMeters(40);

        public static final double turnLowerLimit = 2.5;
        public static final double turnUpperLimit = 90;
        public static final double turnRange = turnUpperLimit - turnLowerLimit;
        public static final double turnSpeed = 0.25;
    }

    public static final class LimelightVision{

        public static final double goalAngleP = 0.5;
        public static final double goalAngleN = -1;
        public static final double deadbandAngle = 1.5;
        public static final double deccelAngle = 30;
        public static final double maxTurn = 0.8;
        public static final double minturn = 0.15;


        public static final double targetHeight = 104; //in
        public static final double cameraHeight = 17; //in
        public static final double cameraAngle = 30.0; //degrees

        public enum TurnDirection {LEFT, RIGHT}

        public enum TurnMode {TELEOP, AUTON}

        public enum DistanceGoal {CLOSE, FAR}

    }



}
