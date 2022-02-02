package frc.robot.io;

public class Axis {
/*
    private Controller mController;
    private AxisID mAxisID;

    public Axis(Controller controller, AxisID axisID){
        mController = controller;
        mAxisID = axisID;
    }

    public int getAxisID(){
        return mAxisID.getID();
    }
*/
    public enum AxisID{

        LEFT_X(0), LEFT_Y(1), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_X(4), RIGHT_Y(5);

        private int mID;

        AxisID(int ID){
            mID = ID;
        }

        public int getID(){
            return mID;
        }


    }

}
