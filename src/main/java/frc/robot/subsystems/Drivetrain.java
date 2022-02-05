// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private CANSparkMax leftLeader, leftFollowerA, leftFollowerB,
          rightLeader, rightFollowerA, rightFollowerB;
  private RelativeEncoder leftEncoder, rightEncoder;
  private DoubleSolenoid shifter;


  public Drivetrain() {
    leftLeader = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftLeaderPort);
    leftFollowerA = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftFollowerAPort);
    leftFollowerB = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.leftFollowerBPort);
    rightLeader = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightLeaderPort);
    rightFollowerA = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightFollowerAPort);
    rightFollowerB = MotorControllerFactory.makeSparkMax(Constants.DriveTrain.rightFollowerBPort);

    rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);


    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DriveTrain.shifterPorts[0], Constants.DriveTrain.shifterPorts[1]);

    rightLeader.setInverted(true);
    rightFollowerA.setInverted(true);
    rightFollowerB.setInverted(true);

    leftFollowerA.follow(leftLeader);
    leftFollowerB.follow(leftLeader);
    rightFollowerA.follow(rightLeader);
    rightFollowerB.follow(rightLeader);

//  Creates two encoder objects for their respective motors
    leftEncoder = leftLeader.getAlternateEncoder(4096);
    rightEncoder = rightLeader.getAlternateEncoder(4096);
    resetEncoders();
    leftEncoder.setInverted(true);

  }

  private void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

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

    leftLeader.set(leftOutput * 0.5);
    rightLeader.set(rightOutput * 0.5);
  }

  public void tankDrive(){
    double left = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double right = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_Y.getID()));

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values
    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

    leftLeader.set(leftOutput * 0.5);
    rightLeader.set(rightOutput * 0.5);
  }

  public void stopDrive(){
    leftLeader.set(0);
    rightLeader.set(0);
  }

  public void vpDrive(double turnSpeed){
    leftLeader.set(turnSpeed);
    rightLeader.set(-turnSpeed);
  }

  public void lowGear(){
    shifter.set(DoubleSolenoid.Value.kReverse);
    System.out.println("Low Gear");
  }

  public void highGear(){
    shifter.set(DoubleSolenoid.Value.kForward);
    System.out.println("High Gear");
  }

  public boolean getLowGear(){
    boolean gear = true;
    if(shifter.get() == DoubleSolenoid.Value.kReverse){
      gear = true; //Low
    } else if(shifter.get() == DoubleSolenoid.Value.kForward){
      gear = false; //High
    }
    return gear;
  }

}
