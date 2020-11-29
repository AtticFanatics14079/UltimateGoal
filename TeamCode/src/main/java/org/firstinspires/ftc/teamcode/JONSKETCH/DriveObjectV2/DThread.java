package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public interface DThread extends Runnable {

    void Stop();
    void start();
    boolean isAlive();

}
