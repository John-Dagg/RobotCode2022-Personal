package frc.robot.photonvision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import static org.photonvision.PhotonUtils.calculateDistanceToTargetMeters;

public class VPPhoton extends SubsystemBase {
    //Subject for removal due to arrival of Limelight 2.0
    //Unfinished


    private PhotonCamera mCamera;
    private PhotonPipelineResult mResult;

    public VPPhoton(){

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
            return calculateDistanceToTargetMeters(Constants.PhotonVision.cameraHeight,
                    Constants.PhotonVision.targetHeightA,
                    Constants.PhotonVision.cameraPitch,
                    getPitch());
        }
        return 0;
    }

    public double configTurn(double Yaw){
        Yaw = getYaw();
        return 2.5 * (Math.abs(Yaw)) / Constants.PhotonVision.turnRange - Constants.PhotonVision.turnLowerLimit / Constants.PhotonVision.turnRange;
    }

}
