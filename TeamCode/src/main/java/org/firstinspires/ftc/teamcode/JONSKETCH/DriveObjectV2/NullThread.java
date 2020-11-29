package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class NullThread implements DThread {

    //Class implementation of DThread

    public NullThread(){}

    public void Stop() {}

    public void start() {}

    public boolean isAlive() {
        return false;
    }

    @Override
    public void run() {
        //Do nothing
    }
}
