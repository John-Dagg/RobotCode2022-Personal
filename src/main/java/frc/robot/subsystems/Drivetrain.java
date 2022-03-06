// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveTrain.*;


public class Drivetrain extends SubsystemBase {

  //Tested and Functional

  private CANSparkMax mLeftLeader, mLeftFollowerA, mLeftFollowerB,
          mRightLeader, mRightFollowerA, mRightFollowerB;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private DoubleSolenoid mShifter;
  private Compressor mComp;
  private Pigeon2 mPigeon;

  private MotorControllerGroup mLeftMotors, mRightMotors;
  private DifferentialDrive mDrive;
  private DifferentialDriveOdometry mOdometry;

  private double positionConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7; //Converts rotations to meters
  private double velocityConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7 / 60; //Converts rpms to meters per second

  private double mYaw, mLeftVolts, mRightVolts;

  private double lastThrottle, lastTurn;

  private DriveState mState;

  public Drivetrain() {

    mLeftLeader = MotorControllerFactory.makeSparkMax(DriveTrain.leftLeaderPort);
    mLeftFollowerA = MotorControllerFactory.makeSparkMax(DriveTrain.leftFollowerAPort);
    mLeftFollowerB = MotorControllerFactory.makeSparkMax(DriveTrain.leftFollowerBPort);
    mRightLeader = MotorControllerFactory.makeSparkMax(DriveTrain.rightLeaderPort);
    mRightFollowerA = MotorControllerFactory.makeSparkMax(DriveTrain.rightFollowerAPort);
    mRightFollowerB = MotorControllerFactory.makeSparkMax(DriveTrain.rightFollowerBPort);

    mRightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightFollowerA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightFollowerB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftFollowerA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftFollowerB.setIdleMode(CANSparkMax.IdleMode.kCoast);

//    mRightLeader.setInverted(true);
//    mRightFollowerA.setInverted(true);
//    mRightFollowerB.setInverted(true);
//
//    mLeftFollowerA.follow(mLeftLeader);
//    mLeftFollowerB.follow(mLeftLeader);
//    mRightFollowerA.follow(mRightLeader);
//    mRightFollowerB.follow(mRightLeader);


    mLeftMotors = new MotorControllerGroup(mLeftLeader, mLeftFollowerA, mLeftFollowerB);
    mRightMotors = new MotorControllerGroup(mRightLeader, mRightFollowerA, mRightFollowerB);
    mLeftMotors.setInverted(true);
    mRightMotors.setInverted(false);

    //Creates an object for interacting with the Pigeon gyro
    mPigeon = new Pigeon2(0);
    resetYaw();

    mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);
    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mPigeon.getYaw()));

    //Creates two encoder objects for their respective motors
    mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    mLeftEncoder.setInverted(false);
    mRightEncoder.setInverted(true);
    resetEncoders();



    mShifter = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, DriveTrain.shifterPorts[0], DriveTrain.shifterPorts[1]);

    lastThrottle = lastTurn = 0.;

    mState = DriveTrain.defaultState;


  }
  //Return objects for interfacing with the motors, encoders, and gyro of the drivetrain

  public void lowGear(){
    mShifter.set(DoubleSolenoid.Value.kReverse);
//    System.out.println("Low Gear"); //For Testing
  }

  public void highGear(){
    mShifter.set(DoubleSolenoid.Value.kForward);
//    System.out.println("High Gear"); //For Testing
  }

  public boolean getLowGear(){
    boolean gear = true;
    if(mShifter.get() == DoubleSolenoid.Value.kReverse){
      gear = true; //Low Gear
    } else if(mShifter.get() == DoubleSolenoid.Value.kForward){
      gear = false; //High Gear
    }
    return gear;
  }



  public MotorControllerGroup getLeftMotors(){
    return mLeftMotors;
  }

  public MotorControllerGroup getRightMotors(){
    return mRightMotors;
  }

  public CANSparkMax getLeftLeader(){
    return mLeftLeader;
  }

  public CANSparkMax getRightLeader(){
    return mRightLeader;
  }

  public RelativeEncoder getLeftEncoder(){
    return mLeftEncoder;
  }

  public RelativeEncoder getRightEncoder(){
    return mRightEncoder;
  }

  public Pigeon2 getPigeon(){return mPigeon;}

  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public void resetYaw(){
    mPigeon.setYaw(0);
  }

  public double getHeading(){
    mYaw = mPigeon.getYaw();
    if(Math.abs(mYaw) > 360){
      resetYaw();
    }
    return mYaw;
  }

  //Ternary operator that sets the percent output to zero if the joystick values aren't above a certain threshold
  private double deadband(double percentOutput){
    return Math.abs(percentOutput) > DriveTrain.deadband ? percentOutput : 0;
  }

  public void masterDrive() {
    switch (mState) {
      case TELE_DRIVE:
        arcadeDrive();
        break;
      case TELE_LIMELIGHT:
        break;
      case AUTO_DRIVE:
        break;
      case AUTO_LIMELIGHT:
        break;
      default:
        break;
    }
  }

  public void arcadeDrive(){
    double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));

    throttle = (throttle == 0) ? 0 : Math.abs(throttle)*throttle;
    turn = (turn == 0) ? 0 : turn/Math.abs(turn)*Math.sqrt(Math.abs(turn));
    turn = ((lastTurn == turn && Math.abs(turn) < 0.33) || turn == 0) ? 0 : turn;
    lastThrottle = (throttle == 0) ? lastThrottle : throttle;
    lastTurn = (turn == 0) ? lastTurn : turn;




    double left = throttle - turn;
    double right = throttle + turn;

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values
    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

//    mLeftLeader.set(leftOutput * 0.5);
//    mRightLeader.set(rightOutput * 0.5);


//    System.out.println(throttle + " | " + turn);
    mDrive.arcadeDrive(-throttle, turn);

    mDrive.feed();
//    printVelocity();
  }

  public void tankDrive(){
    double left = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double right = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_Y.getID()));

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values

    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

    mLeftLeader.set(leftOutput * 0.5);
    mRightLeader.set(rightOutput * 0.5);


    mDrive.tankDrive(left, right);
  }

  public void stopDrive(){
    mLeftLeader.set(0);
    mRightLeader.set(0);
  }



  public void tankDriveVolts(double leftVolts, double rightVolts){
    mLeftVolts = leftVolts;
    mRightVolts = rightVolts;

    System.out.println("Left Volts: " + mLeftVolts);
    System.out.println("Right Volts: " + mRightVolts);

    mLeftMotors.setVoltage(mLeftVolts);
    mRightMotors.setVoltage(mRightVolts);

    mDrive.feed();
  }

  public void printMotors() {
    System.out.println("L1: "+mLeftLeader.getAppliedOutput()
                      +" L2: "+mLeftFollowerA.getAppliedOutput()
                      +" L3: "+mLeftFollowerB.getAppliedOutput()
                      +" R1: "+mRightLeader.getAppliedOutput()
                      +" R2: "+mRightFollowerA.getAppliedOutput()
                      +" R3: "+mRightFollowerB.getAppliedOutput());
  }

  @Override
  public void periodic(){
    mOdometry.update(Rotation2d.fromDegrees(mPigeon.getYaw()) , mLeftEncoder.getPosition() * positionConversion, mRightEncoder.getPosition() * positionConversion);
  }

  public Pose2d getPose(){
    return mOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity() * velocityConversion, mRightEncoder.getVelocity() * velocityConversion);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }





}
