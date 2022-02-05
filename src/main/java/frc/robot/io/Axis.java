package frc.robot.io;

public class Axis {

    public enum AxisID{

        LEFT_X(0), LEFT_Y(1), RIGHT_X(2), RIGHT_Y(3), LEFT_TRIGGER(4), RIGHT_TRIGGER(5);

        private int mID;

        AxisID(int ID){
            mID = ID;
        }

        public int getID(){
            return mID;
        }


    }

}
