package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import java.util.Arrays;

public class ValueStorage {

    private volatile int hardwareCycle = 0;

    private double[][] hardwareValues; //Values returned by hardware

    private double[] runValues; //Values to for the robot to be set to (e.g. velocity for a DMotor).

    public void setup(int size){
        runValues = new double[size];
        hardwareValues = new double[size][1]; //Potentially later change to a large value, currently (9/4/20) having issues with premature calls to hardware.
    }

    public void clear(){
        Arrays.fill(runValues, 0.0);
        Arrays.fill(hardwareValues, null);
    }

    public synchronized void updateCycle() {
        notifyAll();
    }

    public synchronized void waitForCycle() {
        try {
            wait();
            Thread.sleep(0, 300000);
        } catch(Exception e) {
            System.out.println(e);
        }
        //Should allow HardwareThread to access runValues first, however this approach puts every
        //thread exactly 1 cycle behind HardwareThread at all times.

        //Have confirmed that some delay is necessary, <= 0.2 leads to inconsistency.
    }

    //Branched programming to allow for synchronous access to hardwareValues
    public synchronized double[] hardware(boolean writing, double[][] values, int partNum){

        if(writing) {
            for(int i = 0; i < values.length; i++) hardwareValues[i] = values[i].clone();
            return null;
        }

        //Returning clone to prevent threads changing/using the same value at the same time - potentially unnecessary
        return hardwareValues[partNum].clone();
    }

    public synchronized double[] runValues(boolean Writing, double value, int partNum){

        System.out.println("Thread " + Thread.currentThread().getName());

        if(Writing) {
            runValues[partNum] = value;
            return null;
        }

        //Returning clone to prevent threads changing/using the same value at the same time - potentially unnecessary
        return runValues.clone();
    }

    //Used to be used
    /*public synchronized double time(boolean writing, double time){
        if(writing){
            Time = time;
            return 0;
        }
        return Time;
    }
     */
}