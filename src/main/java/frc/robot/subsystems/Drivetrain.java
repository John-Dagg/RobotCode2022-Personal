// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  //Tested and Functional

  private CANSparkMax mLeftLeader, mLeftFollowerA, mLeftFollowerB,
          mRightLeader, mRightFollowerA, mRightFollowerB;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private DoubleSolenoid mShifter;
  private Pigeon2 mPigeon;

  private double yaw;

  public Drivetrain() {
    mLeftLeader = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftLeaderPort);
    mLeftFollowerA = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftFollowerAPort);
    mLeftFollowerB = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftFollowerBPort);
    mRightLeader = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightLeaderPort);
    mRightFollowerA = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightFollowerAPort);
    mRightFollowerB = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightFollowerBPort);

    //Probably should change but for the purpose of testing scary autons
    mRightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);


    mShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DriveTrain.shifterPorts[0], Constants.DriveTrain.shifterPorts[1]);

    mRightLeader.setInverted(true);
    mRightFollowerA.setInverted(true);
    mRightFollowerB.setInverted(true);

    mLeftFollowerA.follow(mLeftLeader);
    mLeftFollowerB.follow(mLeftLeader);
    mRightFollowerA.follow(mRightLeader);
    mRightFollowerB.follow(mRightLeader);

    //Creates two encoder objects for their respective motors
    mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    mRightEncoder.setInverted(true);
    resetEncoders();

    //Creates an object for interacting with the Pigeon gyro
    mPigeon = new Pigeon2(0);
    resetYaw();


  }
  //Return objects for interfacing with the motors, encoders, and gyro of the drivetrain
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

  //Prints rotations of the shaft the encoder is on
  public void printPosition(){
    System.out.println("LEFT ROTATIONS: " + mLeftEncoder.getPosition());
    System.out.println("RIGHT ROTATIONS: " + mRightEncoder.getPosition());
  }

  //Prints rotations per minute of the shaft the encoder is on
  public void printRPM(){
    System.out.println("LEFT RPM: " + mLeftEncoder.getVelocity());
    System.out.println("RIGHT RPM: " + mRightEncoder.getVelocity());
  }

  public void resetYaw(){
    mPigeon.setYaw(0);
  }

  public void printYaw(){
    System.out.println(getYaw());
  }

  public double getYaw(){
    yaw = mPigeon.getYaw();
    if(Math.abs(yaw) > 360){
      resetYaw();
    }
    return yaw;
  }

  //Ternary operator that sets the percent output to zero if the joystick values aren't above a certain threshold
  private double deadband(double percentOutput){
    return Math.abs(percentOutput) > Constants.DriveTrain.deadband ? percentOutput : 0;
  }

  public void arcadeDrive(){
    double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));

    double left = throttle - turn;
    double right = throttle + turn;

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values
    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

    mLeftLeader.set(leftOutput * 0.5);
    mRightLeader.set(rightOutput * 0.5);

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
  }

  public void stopDrive(){
    mLeftLeader.set(0);
    mRightLeader.set(0);
  }

  //For use in PhotonVision. Subject for removal
  public void vpDrive(double turnSpeed){
    mLeftLeader.set(turnSpeed);
    mRightLeader.set(-turnSpeed);
  }

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

}
