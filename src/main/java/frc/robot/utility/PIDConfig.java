package frc.robot.utility;

import com.revrobotics.SparkMaxPIDController;

public class PIDConfig {

    public static void setPID(SparkMaxPIDController leftPIDController, SparkMaxPIDController rightPIDController, double kP, double kI, double kD){
        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);

    }

    public static void setPIDF(SparkMaxPIDController leftPIDController, SparkMaxPIDController rightPIDController, double kP, double kI, double kD, double kFF){
        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        leftPIDController.setFF(kFF);
        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);
        rightPIDController.setFF(kFF);
    }

}
