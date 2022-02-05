package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import static org.photonvision.PhotonUtils.calculateDistanceToTargetMeters;

public class VisionProcessing extends SubsystemBase {

//    private Drivetrain mDrivetrain;
//    private Shooter mShooter;

    private PhotonCamera mCamera;
    private PhotonPipelineResult mResult;

    public VisionProcessing(){
//        mDrivetrain = subsystemA;
//        mShooter = subsystemB;

        mCamera = new PhotonCamera("vptest");
        mCamera.setDriverMode(false);
        mCamera.setPipelineIndex(0);
    }

    private boolean refreshTarget(){
        mResult = mCamera.getLatestResult();
        return mResult.hasTargets();
    }

    public double getPitch() {
        if (refreshTarget()) {
            return mResult.getBestTarget().getPitch();
        }
        return 0;
    }
    public double getYaw(){
        if (refreshTarget()) {
            return mResult.getBestTarget().getYaw();
        }
        return 0;
    }

    public double getRoll(){
        if (refreshTarget()) {
            return mResult.getBestTarget().getYaw();
        }
        return 0;
    }

    public double getArea(){
        if (refreshTarget()) {
            return mResult.getBestTarget().getArea();
        }
        return 0;
    }

    public double getDistance(){
        if (refreshTarget()) {
            return calculateDistanceToTargetMeters(Constants.Vision.cameraHeight,
                    Constants.Vision.targetHeightA,
                    Constants.Vision.cameraPitch,
                    getPitch());
        }
        return 0;
    }

    public double configTurn(double Yaw){
        Yaw = getYaw();
        return 2.5 * (Math.abs(Yaw)) / Constants.Vision.turnRange - Constants.Vision.turnLowerLimit / Constants.Vision.turnRange;
    }

}
