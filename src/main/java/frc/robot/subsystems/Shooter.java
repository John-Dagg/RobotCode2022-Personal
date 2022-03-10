package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;

import static frc.robot.Constants.Shooter.anglerPorts;

public class Shooter extends SubsystemBase {

    //Waiting for Build/Electrical to test

    private TalonFX shooterMotorLeader, shooterMotorFollower;
    private DoubleSolenoid angler;

    private double integratedTicks = 2048;
    private double maxRPM = 6000;

    //Needs testing
    private final double closeVel = -0.67;
    private final double farVel = -1;
    private final double idleVel = 0.00;

    public Shooter(){

        shooterMotorLeader = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterAPort);
        shooterMotorFollower = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterBPort);

        angler = new DoubleSolenoid(anglerPorts[0], PneumaticsModuleType.CTREPCM, anglerPorts[1], anglerPorts[2]);

        shooterMotorFollower.setInverted(true);
        shooterMotorFollower.follow(shooterMotorLeader);
        shooterMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    }

    public void setAnglerLow(){
//        System.out.println(angler.get());
        if (angler.get() != DoubleSolenoid.Value.kReverse) angler.set(DoubleSolenoid.Value.kReverse);
    }

    public void setAnglerHigh(){
//        System.out.println(angler.get());
        if (angler.get() != DoubleSolenoid.Value.kForward) angler.set(DoubleSolenoid.Value.kForward);
    }

    public void setShooterClose(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, closeVel);
    }

    public void setShooterFar(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, farVel);
    }

    public void setShooterIdle(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, idleVel);
    }

    //For use of commands to index the ball once the shooter is at the correct speed
    public double getShooterVel(){
        double ticks = shooterMotorLeader.getSelectedSensorVelocity();
        double RPM = (ticks / integratedTicks) * 10 * 60;
        double percentOutput = RPM / maxRPM;
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
