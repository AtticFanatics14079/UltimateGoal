package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class TimeThread extends Thread implements DThread {

    private double seconds, value, endValue = 0;
    private Active drive;
    private boolean stop = false;

    public TimeThread(double value, double seconds, Active drive){
        this.seconds = seconds;
        this.value = value;
        this.drive = drive;
    }
    public TimeThread(double value, double endValue, double seconds, Active drive){
        this.seconds = seconds;
        this.value = value;
        this.drive = drive;
        this.endValue = endValue;
    }


    //Potentially add a constructor for waiting seconds then setting

    public void run() {
        drive.set(value);
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(!stop) drive.set(endValue);
    }

    public void Stop(){
        stop = true;
    }
}
