package frc.robot.trajectoryfollowing;

public class TrajectoryPoint {

    private final double mDt;
    private final double mPosition;
    private final double mVelocity;
    private final double mHeading;

    public TrajectoryPoint(double dt, double position, double velocity, double heading)
    {
        mDt = dt;
        mPosition = position;
        mVelocity = velocity;
        mHeading = heading;
    }

    public double getDt()
    {
        return mDt;
    }

    public double getPosition()
    {
        return mPosition;
    }

    public double getVelocity()
    {
        return mVelocity;
    }

    public double getHeading()
    {
        return mHeading;
    }
}
