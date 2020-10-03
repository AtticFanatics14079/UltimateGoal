package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class NullThread implements DOThread {

    public NullThread(){}

    public void Stop() {}

    public void start() {}

    public boolean isAlive() {
        return false;
    }
}
