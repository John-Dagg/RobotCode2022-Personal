package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.ControllerFactory;

//import static frc.robot.Constants.Shooter.anglerPorts;

public class Shooter extends SubsystemBase {

    private TalonFX mShooterLeader, shooterMotorFollower;
    private DoubleSolenoid angler;

    private final double integratedTicks = 2048;

    //Acts as universal values to apply to every instance that certain velocities are used for shooting
    //Could be transferred to Constants
    private final double closeVel = 0.68;
    private final double farVel = 0.8;
    private final double lowVel = 0.38;
    private final double idleVel = 0.00;

    private double ticksToRPM = (1 / integratedTicks) * 10 * 60; //ticks per 100ms to RPM
    private double RPMToTicks = integratedTicks / 600; //Multiply by RPMs to get ticks per 100ms
    private final double maxRPM = 5600;
    private double actualPercentConversion = 1. / maxRPM;
    private double RPM, actualPercent, appliedOutput;
    private double deviance = 0, targetPercent = 0.8;


    private int timeOut = 30; //Milliseconds
    private double ticksOut;


    private double gTargetPower, gActualPower, gAppliedPower = 0, gMagnitude, gInterval = 0.005, gDeadband = 0.01, gDeviance;

    public Shooter(){
        SmartDashboard.putNumber("Shooter Speed", closeVel);

        //Creates motor controller objects
        mShooterLeader = ControllerFactory.makeTalonFX(Constants.Shooter.shooterAPort);
        shooterMotorFollower = ControllerFactory.makeTalonFX(Constants.Shooter.shooterBPort);

//        angler = new DoubleSolenoid(anglerPorts[0], PneumaticsModuleType.CTREPCM, anglerPorts[1], anglerPorts[2]);

        shooterMotorFollower.setInverted(true);
        shooterMotorFollower.follow(mShooterLeader);

        //Configures the leader to use the integrated encoder
        mShooterLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, timeOut);

        //Unnecessary - Configures the maximum percentOutputs that can be provided to the motor controller
        mShooterLeader.configNominalOutputForward(0, timeOut);
        mShooterLeader.configNominalOutputReverse(0, timeOut);
        mShooterLeader.configPeakOutputForward(1, timeOut);
        mShooterLeader.configPeakOutputReverse(-1, timeOut);

        /**
         * Configures PID constants for the shooter
         * F (FeedForward) - Value provided by CTRE
         * P (Proportional) - An increased value causes the motor controller to more aggressively approach the setpoint but too high will cause oscillations and overshooting
         * I (Integral) - Increases the affect of the motor controller overtime leading to more or less oscillations. Increases accuracy but can lead to more oscillations and integral windup
         * D (Derivative) - Provides a dampening affect to the motor controller to prevent oscillations but may lead to an inaccurate setpoint
         */

        mShooterLeader.config_kF(0, 1023.0/20660.0, timeOut); //Specific Value provided by CTRE
        mShooterLeader.config_kP(0, 0.6, timeOut);
        mShooterLeader.config_kI(0, 0, timeOut);
        mShooterLeader.config_kD(0, 0.5, timeOut);

        appliedOutput = targetPercent;

    }

    //Method used to print different variable to check if things are running as intended
    @Override
    public void periodic(){
//        System.out.println("Actual Percent Output: " + MathEqs.roundCustom(gActualPower) +
//                           " | Applied Percent Output: " + MathEqs.roundCustom(gAppliedPower) +
//                           " | Target Percent Output: " + MathEqs.roundCustom(gTargetPower));
    }

    public void PIDshooter(double rpm){
        ticksOut = rpm * RPMToTicks;
        mShooterLeader.set(TalonFXControlMode.Velocity, -ticksOut);
    }

    /**
     * @param setpointRPM
     * @param realRPM
     * @param error
     * @return Returns whether the rpms of the shooter are within a certain margin of error
     */
    public boolean checkRPM(double setpointRPM, double realRPM, double error){
        return Math.abs(realRPM) > (setpointRPM - error);
    }

    public void rpmShootLow(){
        PIDshooter(2000);
    }

    public void rpmShootClose(){
        PIDshooter(3750);
    }

    public void rpmShootFar() {
        PIDshooter(5000);
    }

    public void setShooterIdle(){
        mShooterLeader.set(TalonFXControlMode.PercentOutput, idleVel);
    }

    /**
     * Alternative to PID control developed to assist the motor controller of achieving a certain velocity regardless of voltage
     * @param gTargetPower
     */
    public void setShooterGain(double gTargetPower){

        this.gTargetPower = gTargetPower;

        mShooterLeader.set(TalonFXControlMode.PercentOutput, -gAppliedPower);

        RPM = mShooterLeader.getSelectedSensorVelocity() * ticksToRPM;
        gActualPower = Math.abs(RPM * actualPercentConversion);

        gDeviance = gTargetPower - gActualPower;

        if (Math.abs(gDeviance) > gDeadband) gMagnitude = Math.signum(gDeviance)*gInterval;
        else gMagnitude = 0;
        gAppliedPower = gTargetPower + gMagnitude;
    }

    public void shootLow(){
        setShooterGain(lowVel);
    }

    public void shootClose(){
        setShooterGain(closeVel);
    }

    public void shootFar(){
        setShooterGain(farVel);
    }

    public void printRPM(){
        double RPM = mShooterLeader.getSelectedSensorVelocity() * ticksToRPM;
        if (RPM == 0.0) return; //Exits method if true
        System.out.println("RPM: " + RPM);
        SmartDashboard.putNumber("RPM", RPM);
    }

    /**
     *  Below are old methods that follow the initial thought process in creating the shooter class and were used for testing
     */

    public void setShooter(){
        //Uses Smart Dashboard to receive information about shooter speed to adjust it on the fly without redeploying
        double vel = SmartDashboard.getNumber("Shooter Speed", closeVel);
        appliedOutput = Math.abs(vel);

        mShooterLeader.set(TalonFXControlMode.PercentOutput, -appliedOutput);
    }

    public void setShooterVel(double speed){
        mShooterLeader.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getShooterRPM(){
        return mShooterLeader.getSelectedSensorVelocity() * ticksToRPM;
    }

    public TalonFX getShooterLeader(){
        return mShooterLeader;
    }

    //For testing whether the shooter motors work
    public void shooterTest(){
        mShooterLeader.set(TalonFXControlMode.PercentOutput, Constants.operatorController.getRawAxis(Axis.LEFT_Y.getID()));
    }

    //For changing the state of the angler back when it was on the robot
    /*
    public void setAnglerLow(){
        if (angler.get() != DoubleSolenoid.Value.kReverse) angler.set(DoubleSolenoid.Value.kReverse);
    }

    public void setAnglerHigh(){
        if (angler.get() != DoubleSolenoid.Value.kForward) angler.set(DoubleSolenoid.Value.kForward);
    }

     */

}
