package frc.robot.trajectoryfollowing;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Trajectory {

    private ArrayList<TrajectoryPoint> mPoints = new ArrayList<>();

    private ArrayList<Double> mVelocities = new ArrayList<>();

    /**
     * Creates a new trajectory with the units representing inches
     *
     * @param file The trajectory config to read from
     */
    public Trajectory(File file, TargetMotorController targetMotorController)
    {
        this(file, targetMotorController, 1);
    }

    public Trajectory(File file, TargetMotorController targetMotorController, int unitsPerInch)
    {
        System.out.println("FILE: "+file.getName());
        try
        {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
            String line;
            String[] properties;

            // Gets us past the header so we only read the values and not the labels
            bufferedReader.readLine();

            while ((line = bufferedReader.readLine()) != null)
            {
                properties = line.split(",");

                double dt = Double.parseDouble(properties[0]);

                double position = Double.parseDouble(properties[3]) * unitsPerInch;
                double rawVelocity = Double.parseDouble(properties[4]) * unitsPerInch;

                // CTRE motor controllers measure velocity in units per 100ms
                double velocity = targetMotorController == TargetMotorController.REV ? rawVelocity / 10 : rawVelocity;

                //Adds all velocities to an array
                mVelocities.add(velocity);

                double heading = Double.parseDouble(properties[7]) * unitsPerInch;

                mPoints.add(new TrajectoryPoint(dt, position, velocity, heading));
            }

            bufferedReader.close();
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public ArrayList<TrajectoryPoint> getPoints()
    {
//        System.out.println("LENGTH: "+mPoints.size());
        return mPoints;
    }

    public enum TargetMotorController
    {
        CTRE, REV
    }

    //Finds max velocity
    public double findMaxVelocity(){
        double maxVel = 0;
        for(int i = 0; i < mVelocities.size(); i++){
            if (Math.abs(mVelocities.get(i)) > maxVel){
                maxVel = mVelocities.get(i);
            }
        }
        return maxVel;
    }
}
