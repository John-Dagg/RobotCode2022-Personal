package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;

import static frc.robot.Constants.Shooter.anglerPorts;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorLeader, shooterMotorFollower;
    private DoubleSolenoid angler;
    private PIDController mController;

    private final double integratedTicks = 2048;

    private final double closeVel = -0.68;
    private final double farVel = -0.75;
    private final double idleVel = 0.00;

    private double RPMgoal;

    public Shooter(){
        SmartDashboard.putNumber("Shooter Speed", closeVel);

        shooterMotorLeader = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterAPort);
        shooterMotorFollower = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterBPort);

        angler = new DoubleSolenoid(anglerPorts[0], PneumaticsModuleType.CTREPCM, anglerPorts[1], anglerPorts[2]);

        shooterMotorFollower.setInverted(true);
        shooterMotorFollower.follow(shooterMotorLeader);
        shooterMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        mController = new PIDController(0.1, 0, 0);
    }

    public void PIDshooter(){
        RPMgoal = 4000;
        double setpoint = (RPMgoal / 600) * integratedTicks;
        double value = mController.calculate(shooterMotorLeader.getSelectedSensorVelocity(), setpoint);
        System.out.println("PID Controller Value: " + value);
        shooterMotorLeader.set(TalonFXControlMode.Velocity, value);
    }

    public void setAngle(){
        if (angler.get() != DoubleSolenoid.Value.kForward) angler.set(DoubleSolenoid.Value.kForward);
        if (angler.get() != DoubleSolenoid.Value.kReverse) angler.set(DoubleSolenoid.Value.kReverse);
    }

    public void setAnglerLow(){
        if (angler.get() != DoubleSolenoid.Value.kReverse) angler.set(DoubleSolenoid.Value.kReverse);
    }

    public void setAnglerHigh(){
        if (angler.get() != DoubleSolenoid.Value.kForward) angler.set(DoubleSolenoid.Value.kForward);
    }

    public void setShooterClose(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, closeVel);
        setAnglerLow();
    }

    public void setShooterFar(){
        setAnglerHigh();
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, farVel);
        printRPM();
    }

    public void setShooterVel(double speed){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, speed);
        printRPM();
    }

    public void setShooter(){
        double vel = SmartDashboard.getNumber("Shooter Speed", closeVel);
        System.out.println("Shooter Vel" + getShooterVel());
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, vel);
        printRPM();
    }

    public void setShooterIdle(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, idleVel);
    }

    public void printRPM(){
        double ticks = shooterMotorLeader.getSelectedSensorVelocity();
        double RPM = (ticks / integratedTicks) * 10 * 60; //Converts to RPM from ticks per 100ms
        System.out.println("RPM: " + RPM);
        SmartDashboard.putNumber("RPM", RPM);
    }

    @Override
    public void periodic(){
        printRPM();
    }

    //For use of commands to index the ball once the shooter is at the correct speed
    public double getShooterVel(){
        double ticks = shooterMotorLeader.getSelectedSensorVelocity();
        double RPM = (ticks / integratedTicks) * 10 * 60; //Converts to RPM from ticks per 100ms
        double maxRPM = 6000;
        System.out.println("RPM: " + RPM);
        double percentOutput = RPM / maxRPM;
//        System.out.println("Shooter Percent Output: " + percentOutput);
        return percentOutput;
    }

    //For use of commands to index the ball into the shooter before the falcons on the shooter reach full speed
    public double getShooterCloseVel(){
        return closeVel - 0.1;
    }

    public double getShooterFarVel(){
        return farVel - 0.1;
    }

    public TalonFX getShooterLeader(){
        return shooterMotorLeader;
    }

    public void shooterTest(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, Constants.operatorController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
    }





}
