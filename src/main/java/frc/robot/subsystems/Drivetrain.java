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

    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DriveTrain.shifterA, Constants.DriveTrain.shifterB);

    leftLeader.setInverted(true);
    leftFollowerA.setInverted(true);
    leftFollowerB.setInverted(true);

    leftFollowerA.follow(leftLeader);
    leftFollowerB.follow(leftLeader);
    rightFollowerA.follow(rightLeader);
    rightFollowerB.follow(rightLeader);

//  Creates two encoder objects for their respective motors
    leftEncoder = leftLeader.getAlternateEncoder(2048);
    rightEncoder = rightLeader.getAlternateEncoder(2048);
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

    double left = throttle + turn;
    double right = throttle - turn;

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values
    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

    leftLeader.set(leftOutput);
    rightLeader.set(rightOutput);
  }

  public void tankDrive(){
    double left = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    double right = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_Y.getID()));

    //Ternary operators that ensure the values supplied to the SparkMaxes are within the acceptable range.
    //Math.max returns the greater of the two values
    //Math.min returns the lower of the two values
    double leftOutput = left < 0 ? Math.max(left, -1) : Math.min(left, 1);
    double rightOutput = right < 0 ? Math.max(right, -1) : Math.min(right, 1);

    leftLeader.set(leftOutput);
    rightLeader.set(rightOutput);
  }

  public void lowGear(){
    shifter.set(DoubleSolenoid.Value.kReverse);
  }

  public void highGear(){
    shifter.set(DoubleSolenoid.Value.kForward);
  }

}
