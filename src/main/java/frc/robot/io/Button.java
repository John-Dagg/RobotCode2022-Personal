package frc.robot.io;

public class Button {

    public enum ButtonID {

        A(1), B(2), X(3), Y(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), SELECT(7), START(8), LEFT_STICK(9), RIGHT_STICK(10);

        int mID;

        ButtonID(int ID){
            mID = ID;
        }

        public int getID(){
            return mID;
        }

    }
}
