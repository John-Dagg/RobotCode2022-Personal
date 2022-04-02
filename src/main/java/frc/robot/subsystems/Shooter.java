package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MathEqs;
import frc.robot.utility.MotorControllerFactory;

import static frc.robot.Constants.Shooter.anglerPorts;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorLeader, shooterMotorFollower;
    private DoubleSolenoid angler;

    private final double integratedTicks = 2048;

    private final double closeVel = 0.68;
    private final double farVel = 0.8;
    private final double lowVel = 0.38;
    private final double idleVel = 0.00;

    private double ticksToRPM = (1 / integratedTicks) * 10 * 60; //ticks per 100ms to RPM
    private final double maxRPM = 5600;
    private double actualPercentConversion = 1. / maxRPM;
    double RPM, actualPercent, appliedOutput;
    double deviance = 0;
    double targetPercent = 0.8;


    private double gTargetPower, gActualPower, gAppliedPower = 0, gMagnitude, gInterval = 0.005, gDeadband = 0.01, gDeviance;

    public Shooter(){
        SmartDashboard.putNumber("Shooter Speed", closeVel);

        shooterMotorLeader = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterAPort);
        shooterMotorFollower = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterBPort);

        angler = new DoubleSolenoid(anglerPorts[0], PneumaticsModuleType.CTREPCM, anglerPorts[1], anglerPorts[2]);

        shooterMotorFollower.setInverted(true);
        shooterMotorFollower.follow(shooterMotorLeader);
        shooterMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        appliedOutput = targetPercent;

    }

    @Override
    public void periodic(){
//        printRPM();
//        System.out.println("Actual Percent Output: " + MathEqs.roundCustom(gActualPower) +
//                           " | Applied Percent Output: " + MathEqs.roundCustom(gAppliedPower) +
//                           " | Target Percent Output: " + MathEqs.roundCustom(gTargetPower));
    }

    public void calcShooter(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, -appliedOutput);

        actualPercent = Math.abs(shooterMotorLeader.getSelectedSensorVelocity() * ticksToRPM * actualPercentConversion);
        deviance = targetPercent - actualPercent;
        appliedOutput = targetPercent + deviance;
    }

    public void setShooterGain(double gTargetPower){

        this.gTargetPower = gTargetPower;

        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, -gAppliedPower);

        RPM = shooterMotorLeader.getSelectedSensorVelocity() * ticksToRPM;
        gActualPower = Math.abs(RPM * actualPercentConversion);

        gDeviance = gTargetPower - gActualPower;

        if (Math.abs(gDeviance) > gDeadband) {
            gMagnitude = Math.signum(gDeviance)*gInterval;
        }
        else {
            gMagnitude = 0;
        }
        gAppliedPower = gTargetPower + gMagnitude;
//        System.out.println("Per Out: " + gAppliedPower + " | Leader Current: " + shooterMotorLeader.getSupplyCurrent() + " | Follower Current: " + shooterMotorFollower.getSupplyCurrent());

    }

    public void shootLow(){
        setShooterGain(lowVel);
        if (angler.get() != DoubleSolenoid.Value.kReverse) setAnglerLow();
    }

    public void shootClose(){
        setShooterGain(closeVel);
        if (angler.get() != DoubleSolenoid.Value.kForward) setAnglerHigh();

    }

    public void shootFar(){
        setShooterGain(farVel);
        if (angler.get() != DoubleSolenoid.Value.kForward) setAnglerHigh();
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
        RPM = shooterMotorLeader.getSelectedSensorVelocity() * ticksToRPM;
        actualPercent = RPM * actualPercentConversion;
//        printRPM();
    }

    public void setShooter(){
        double vel = SmartDashboard.getNumber("Shooter Speed", closeVel);
        /*
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, vel);
        RPM = shooterMotorLeader.getSelectedSensorVelocity() * ticksToRPM;
        actualPercent = RPM * actualPercentConversion;

         */
        appliedOutput = Math.abs(vel);
        RPM = shooterMotorLeader.getSelectedSensorVelocity() * ticksToRPM;
        actualPercent = Math.abs(RPM * actualPercentConversion);
//
//        double deviance = (appliedOutput - actualPercent);
//        if (appliedOutput - gain > actualPercent + 0.01) {
//            gain += 0.005;
//        }
//        else if (appliedOutput + gain < actualPercent - 0.01) {
//            gain -= 0.005;
//        }


        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, -appliedOutput);
//        printRPM();
    }



    public void setShooterIdle(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, idleVel);
    }

    public void printRPM(){
        double ticks = shooterMotorLeader.getSelectedSensorVelocity();
        double RPM = (ticks / integratedTicks) * 10 * 60; //Converts to RPM from ticks per 100ms
        if (RPM == 0.0) return;
        System.out.println("RPM: " + RPM);
        SmartDashboard.putNumber("RPM", RPM);
    }



    //For use of commands to index the ball once the shooter is at the correct speed
    public double getShooterVel(){
        return gActualPower;
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
