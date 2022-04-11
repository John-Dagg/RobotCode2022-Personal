// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Holds all constants for motor ports, solenoid ports, enumeration states, limelight constants, and auton constants
 */
public final class Constants {

    public static final Joystick driverController = new Joystick(0);
    public static final Joystick operatorController = new Joystick(1);

    public enum IntakeState {IN, OUT}
    public enum HoodState {LOW, HIGH}

    public static final class Auton {

        public static final class LowGear {

            public static final double Low_ks = 0.206; //Volts
            public static final double Low_kv = 1.8559; //Volt seconds per meter
            public static final double Low_ka = 0.58011; //Volt seconds squared per meter

            public static final double Low_kP = 2.9; //Proportional Gain
        }

        public static final class HighGear {

            public static final double High_ks = 0.2334;
            public static final double High_kv = 2.1969;
            public static final double High_ka = 0.5125;

            public static final double High_kP = 3.1031;
        }

        public static final double robotWidth = 0.74325;
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

        public enum DriveState {TELE_DRIVE_INTAKE, TELE_DRIVE_SHOOTER, TELE_LIMELIGHT, LIMELIGHT_DRIVE, AUTO_DRIVE, AUTO_LIMELIGHT}

        public enum ShiftState {LOW_GEAR, HIGH_GEAR}

        public static final DriveState defaultState = DriveState.TELE_DRIVE_INTAKE;

    }

    public static final class Intake {

        public static final int intakeMotorPort = 4;

        public static final int[] fourBarPorts = {13, 6, 7};
    }

    public static final class Indexer {

        public static final int indexerPort = 8;
    }

    public static final class Shooter {

        public static final int shooterAPort = 9;
        public static final int shooterBPort = 10;

//        public static final int[] anglerPorts = {13, 6, 7};
    }

    public static final class Climber{

        public static final int climberPortA = 11;
        public static final int climberPortB = 12;

        public static final int[] solenoidPorts = {13, 0, 1};
//        public static final int[] brakePorts = {13, 2, 3};

        public static final double deadband = 0.1;
    }

    public static final class LimelightVision{

        public static final class LowGear {
            public static final double deadbandAngle_Low = 2.;
            public static final double deccelAngle_Low = 30.;
            public static final double maxTurn_Low = 0.8;
            public static final double minturn_Low = 0.3;
        }

        public static final class HighGear {
            public static final double deadbandAngle_High = 2.;
            public static final double deccelAngle_High = 30.;
            public static final double maxTurn_High = 0.65;
            public static final double minturn_High = 0.25;
        }

        public static final double targetHeight = 104; //in
        public static final double cameraHeight = 17; //in
        public static final double cameraAngle = 30.0; //degrees

        public enum TurnDirection {LEFT, RIGHT, NONE}

        public enum TurnMode {TELEOP, AUTON}

    }



}
