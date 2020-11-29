package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareThread extends Thread {

    ElapsedTime time;
    private ValueStorage vals;
    double[][] hardwareVals; //Holds the values received from hardware of each part.
    double[] lastRun; //Previous run values.
    public Configuration config;
    private volatile boolean stop;
    private boolean setTime = false;
    public double voltMult = 1, lastTime = 0;

    public HardwareThread(HardwareMap hwMap, ValueStorage vals, Configuration configuration){
        config = configuration;
        config.Configure(hwMap, vals);
        int size = config.hardware.size();
        this.vals = vals;
        this.vals.setup(size);
        hardwareVals = new double[size][];
        lastRun = new double[size];
        //voltMult = 13.0/config.voltSense.getVoltage();
        config.setBulkCachingManual(true);
    }

    public void run(){

        time = new ElapsedTime();

        try {
            while(!stop) {

                readHardware(); //Longest section by a ridiculous margin (about 90% of time)

                System.out.println("Hardware cycle: " + time.milliseconds());
                vals.updateCycle();
                //Should allow every other thread to simply wait for cycle. Consider moving this or adding a sleep to prevent runValues being off by a cycle.

                runHardware(vals.runValues(false, 0, 0));
            }
        }
        catch(Exception e) {}
        finally {
            for(DriveObject d : config.hardware) {
                d.endThreads();
            }
            vals.clear();
        }
    }

    private void readHardware(){

        config.clearBulkCache(); //Miniscule time

        for(int i = 0; i < hardwareVals.length; i++) {
            hardwareVals[i] = config.hardware.get(i).getHardware(); //Majority of time in this loop
        }

        vals.hardware(true, hardwareVals, 0);
    }

    private void runHardware(double[] Values) {

        for(int i = 0; i < this.hardwareVals.length; i++) {
            if(lastRun[i] != Values[i])
                config.hardware.get(i).setHardware(Values[i]);
        }

        lastRun = Values;
    }

    public void Stop(){
        stop = true;
    }
}
