package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareThread extends Thread {

    ElapsedTime time;
    private ValueStorage vals;
    double[][] hardwareVals; //Holds the values received from hardware of each part.
    double[] lastRun; //Previous run values.
    public Configuration config;
    private volatile boolean stop, resetIMU = false;
    private boolean setTime = false; //Vestigial variable
    public double voltMult = 1, lastTime = 0; //Vestigial variables

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
                System.out.println("Hardware cycle: " + time.milliseconds());
                vals.updateCycle(); //Should allow every other thread to simply wait for cycle. Consider moving this or adding a sleep to prevent runValues being off by a cycle.
                System.out.println("After update: " + time.milliseconds());

                readHardware(); //Longest section by a ridiculous margin (about 90% of time).
                System.out.println("After read: " + time.milliseconds());

                runHardware(vals.runValues(false, 0, 0));
                System.out.println("After run: " + time.milliseconds());

                //updateHardware();
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
        System.out.println("After clearing cache: " + time.milliseconds());

        for(int i = 0; i < hardwareVals.length; i++) {
            if(config.hardware.get(i) instanceof DIMU && !((DIMU) config.hardware.get(i)).gettingInput) hardwareVals[i] = new double[]{0, 0, 0};
            else hardwareVals[i] = config.hardware.get(i).getHardware(); //Majority of time in this loop
            System.out.println("After reading part " + i + ": " + time.milliseconds());
        }

        System.out.println("After reading: " + time.milliseconds());

        vals.hardware(true, hardwareVals, 0);
    }

    private void runHardware(double[] Values) {

        for(int i = 0; i < this.hardwareVals.length; i++) {
            if(config.hardware.get(i) instanceof Active && lastRun[i] != Values[i]) {
                ((Active) (config.hardware.get(i))).setHardware(Values[i]);
            }
            //instanceof and typecasting allows for sensors to not include setHardware.
        }

        lastRun = Values;
    }

    private void updateHardware() {
        for(int i = 0; i < this.hardwareVals.length; i++) {
            if(config.hardware.get(i) instanceof DIMU && resetIMU) {
                ((DIMU) config.hardware.get(i)).resetIMU();
                resetIMU = false;
            }
            //instanceof and typecasting allows for sensors to not include setHardware.
        }
    }

    public void resetIMU() {
        resetIMU = true;
    }

    public void Stop(){
        stop = true;
    }
}
