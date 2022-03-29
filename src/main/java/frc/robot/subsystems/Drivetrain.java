// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Axis;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.utility.MotorControllerFactory;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveTrain.*;

import static frc.robot.Constants.DriveTrain.DriveState.*;
import static frc.robot.Constants.DriveTrain.ShiftState.*;
import static frc.robot.Constants.DriveTrain.defaultState;
import static frc.robot.Constants.DriveTrain.shifterPorts;
import java.util.Arrays;


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
  //SysID Gear Ratio (High Gear) - 1 / 8.15
  private final double positionConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7 * (7.3 / 8.5); //Converts rotations to meters
  private final double velocityConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7 / 60 * (7.3 / 8.5); //Converts rpms to meters per second

  private double mYaw, mLeftVolts, mRightVolts;

  private double lastThrottle, lastTurn, limelightThrottle, limelightTurn, limelightRawTurn;
  private double yaw;
  private double limelightGain;

  public DriveState mState;
  public ShiftState mShiftState;

  private VPLimelight mLimelight;

  private final double distanceClose = 62; //inches
  private final double distanceFar = 178; //inches
  private final double band = 10;

  private double initOffset = 0;


  public Drivetrain(VPLimelight subsystemA) {
    SmartDashboard.putNumber("Distance", 0);

    mLimelight = subsystemA;

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

    mLeftMotors = new MotorControllerGroup(mLeftLeader, mLeftFollowerA, mLeftFollowerB);
    mRightMotors = new MotorControllerGroup(mRightLeader, mRightFollowerA, mRightFollowerB);
    mLeftMotors.setInverted(true);
    mRightMotors.setInverted(false);

    //Creates an object for interacting with the Pigeon gyro
    mPigeon = new Pigeon2(0);
    resetYaw();


    //Creates two encoder objects for their respective motors
//    mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
//    mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    mLeftEncoder = mLeftLeader.getEncoder();
    mRightEncoder = mRightLeader.getEncoder();

//    mLeftEncoder.setInverted(false);
//    mRightEncoder.setInverted(true);
    resetEncoders();

    mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);
    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mPigeon.getYaw()));

    mShifter = new DoubleSolenoid(shifterPorts[0], PneumaticsModuleType.CTREPCM, shifterPorts[1], shifterPorts[2]);

    lastThrottle = lastTurn = 0.;

    mState = defaultState;


  }

  public void masterDrive() {
    switch (mState) {
      case TELE_DRIVE_INTAKE:
        arcadeDriveIntake();
//        mLimelight.steadyArray();
//        System.out.println("Distance (in) " + mLimelight.calcDistance());
        break;
      case TELE_DRIVE_SHOOTER:
        arcadeDriveShooter();
        break;
      case TELE_LIMELIGHT:
        limelightDrive();
        break;
      case LIMELIGHT_DRIVE:
        limelightDrive();
      case AUTO_DRIVE:
//        System.out.println("Left Volts: " + mLeftVolts);
//        System.out.println("Right Volts: " + mRightVolts);
        break;
      case AUTO_LIMELIGHT:
        limelightDrive();
//        System.out.println("Left Volts: " + mLeftVolts);
//        System.out.println("Right Volts: " + mRightVolts);
        break;
      default:
        break;
    }
    double dist = mLimelight.calcDistance();
    SmartDashboard.putNumber("Distance", dist);

    if ((Arrays.asList(new DriveState[] {TELE_LIMELIGHT, LIMELIGHT_DRIVE, AUTO_LIMELIGHT, AUTO_DRIVE}).contains(mState))) {
      mLimelight.steadyArray();
//      System.out.println("Distance (in) " + mLimelight.calcDistance());
    }
    else mLimelight.offArray();

//    System.out.println("Re (m): "+leftWheelsPosition()+" Le (m): "+rightWheelsPosition());

//    System.out.println(mState);
  }

  @Override
  public void periodic(){
    mOdometry.update(Rotation2d.fromDegrees(mPigeon.getYaw()), -mLeftEncoder.getPosition() * positionConversion, mRightEncoder.getPosition() * positionConversion);
    var translation = mOdometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("X", translation.getX());
    SmartDashboard.putNumber("Y", translation.getY());



    if ((Arrays.asList(new DriveState[] {AUTO_DRIVE, AUTO_LIMELIGHT}).contains(mState))) {
      masterDrive();
    }

//    System.out.println("RIGHT POSITION: " + rightWheelsPosition() + " | LEFT POSITION: " + leftWheelsPosition());
  }


  public void lowGear(){
    mShifter.set(DoubleSolenoid.Value.kReverse);
    mShiftState = LOW_GEAR;
  }

  public void highGear(){
    mShifter.set(DoubleSolenoid.Value.kForward);
    mShiftState = HIGH_GEAR;
  }

  public ShiftState getShiftState(){
    if (mShifter.get() == DoubleSolenoid.Value.kReverse) mShiftState = LOW_GEAR;
    else mShiftState = HIGH_GEAR;
    return mShiftState;
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


  public void setIdleState(CANSparkMax.IdleMode state) {
    if (!(mRightLeader.getIdleMode() == state)) {
      mRightLeader.setIdleMode(state);
      mRightFollowerA.setIdleMode(state);
      mRightFollowerB.setIdleMode(state);
    }
    if (!(mLeftLeader.getIdleMode() == state)) {
      mLeftLeader.setIdleMode(state);
      mLeftFollowerA.setIdleMode(state);
      mLeftFollowerB.setIdleMode(state);
    }
  }

  public void setState(DriveState state){
    mState = state;
  }

  public void setShooterDrive(){
    setState(TELE_DRIVE_SHOOTER);
  }

  public void setIntakeDrive(){
    setState(TELE_DRIVE_INTAKE);
  }

  public void defaultState(){
    mState = defaultState;
  }

  public void toggleDriveState(){
    if (mState != LIMELIGHT_DRIVE) {
      mState = LIMELIGHT_DRIVE;
    }
    if (mState != TELE_DRIVE_INTAKE) {
      mState = TELE_DRIVE_INTAKE;
    }

  }

  public void toggleArcadeStyle(){
    if (mState != TELE_DRIVE_INTAKE){
      mState = TELE_DRIVE_INTAKE;
      System.out.println("Intake Arcade Drive");
    }
    if (mState != TELE_DRIVE_SHOOTER){
      mState = TELE_DRIVE_SHOOTER;
      System.out.println("Shooter Tele Drive");
    }
  }

  public void arcadeDriveShooter(){
    double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));

//    throttle = (throttle == 0) ? 0 : Math.abs(throttle)*throttle;
//    turn = (turn == 0) ? 0 : turn/Math.abs(turn)*Math.sqrt(Math.abs(turn));
//    turn = ((lastTurn == turn && Math.abs(turn) < 0.33) || turn == 0) ? 0 : turn;

    mDrive.arcadeDrive(throttle, turn);
    lastThrottle = (throttle == 0) ? lastThrottle : throttle;
    lastTurn = (turn == 0) ? lastTurn : turn;
    mDrive.feed();

    double xOffset = mLimelight.getxOffset();
    if (xOffset > 0){
//      System.out.println(xOffset + " degrees to the RIGHT(?)");
      SmartDashboard.putNumber("Degrees to the RIGHT", xOffset);
    } else {
//      System.out.println(xOffset + " degrees to the LEFT(?)");
      SmartDashboard.putNumber("Degrees to the LEFT", xOffset);
    }

  }


  public void arcadeDriveIntake(){
    double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));

//    System.out.println("Left Position (m) :" + mLeftEncoder.getPosition() * positionConversion);
//    System.out.println("Right Position (m) :" + mRightEncoder.getPosition() * positionConversion);

//    System.out.println("Yaw: " + mPigeon.getYaw());

    mDrive.arcadeDrive(-throttle, turn);
    lastThrottle = (throttle == 0) ? lastThrottle : throttle;
    lastTurn = (turn == 0) ? lastTurn : turn;
    mDrive.feed();

    double xOffset = mLimelight.getxOffset();
    if (xOffset > 0){
//      System.out.println(xOffset + " degrees to the RIGHT(?)");
      SmartDashboard.putNumber("Degrees to the RIGHT", xOffset);
    }
    if (xOffset < 0){
//      System.out.println(xOffset + " degrees to the LEFT(?)");
      SmartDashboard.putNumber("Degrees to the LEFT", xOffset);
    }

  }

  public void limelightDrive(){
    double[] values = mLimelight.getValues();
    mDrive.arcadeDrive(values[0], values[1]);
    mDrive.feed();
  }

  public void autonDrive(double thottle, double turn){
    mDrive.arcadeDrive(thottle, turn);
    mDrive.feed();
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

//    System.out.println("Le: " + leftWheelsPosition() + " Re: " + rightWheelsPosition());

//    System.out.println("Yaw: " + mPigeon.getYaw());

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


  /***
   * Returns the position of the left wheels in meters
   * @return Meters
   */

  public double leftWheelsPosition(){
    return -mLeftEncoder.getPosition() * positionConversion;
  }

  /***
   * Returns the position of the right wheels in meters
   * @return Meters
   */

  public double rightWheelsPosition(){
//    System.out.println("Right distance: "+mRightEncoder.getPosition() * positionConversion);
    return mRightEncoder.getPosition() * positionConversion;
  }

  public Pose2d getPose(){
    return mOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(-mLeftEncoder.getVelocity() * velocityConversion, mRightEncoder.getVelocity() * velocityConversion);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
//    resetYaw();
    initOffset = mPigeon.getYaw();
    System.out.println("Starting Degrees: " + Rotation2d.fromDegrees(mPigeon.getYaw()));
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(mPigeon.getYaw()));
  }





}
