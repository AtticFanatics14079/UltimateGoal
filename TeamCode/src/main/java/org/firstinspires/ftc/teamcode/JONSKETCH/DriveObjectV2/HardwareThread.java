package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareThread extends Thread implements SharedObjects {

    ElapsedTime time;
    double[][] hardwareVals; //Holds the values received from hardware of each part.
    double[] lastRun; //Previous run values.
    public Configuration config;
    private volatile boolean stop = false, resetIMU = false;
    private boolean setTime = false; //Vestigial variable
    public double voltMult = 1, lastTime = 0; //Vestigial variables

    public HardwareThread(HardwareMap hwMap, Configuration configuration){
        config = configuration;
        config.Configure(hwMap, vals);
        int size = hardware.size();
        vals.setup(size);
        hardwareVals = new double[size][];
        lastRun = new double[size];
        config.setBulkCachingManual(true);
    }

    public ValueStorage getVals() {
        return vals;
    }

    public void run(){

        time = new ElapsedTime();

        try {
            while(!stop) {
                System.out.println("Hardware cycle: " + time.milliseconds());
                vals.updateCycle(); //Should allow every other thread to simply wait for cycle. Consider moving this or adding a sleep to prevent runValues being off by a cycle.

                readHardware(); //Longest section by a ridiculous margin (about 90% of time).

                runHardware(vals.runValues(false, 0, 0));
            }
        }
        catch(Exception e) {}
        finally {
            for(DriveObject d : hardware) {
                d.endThreads();
            }
            System.out.println("Hardware Time 1: " + time.milliseconds());
            vals.updateCycle();
            System.out.println("Hardware Time 2: " + time.milliseconds());
            vals.clear();
        }
    }

    private void readHardware(){

        config.clearBulkCache(); //Minuscule time

        for(int i = 0; i < hardwareVals.length; i++) {
            //if(hardware.get(i) instanceof DIMU && !((DIMU) hardware.get(i)).gettingInput) hardwareVals[i] = new double[]{};
            //else if(hardware.get(i) instanceof DDistanceSensor && !((DDistanceSensor) hardware.get(i)).gettingInput) hardwareVals[i] = new double[]{};
            hardwareVals[i] = hardware.get(i).getHardware(); //Majority of time in this loop
        }

        vals.hardware(true, hardwareVals, 0);
    }

    private void runHardware(double[] Values) {

        for(int i = 0; i < this.hardwareVals.length; i++) {
            if(hardware.get(i) instanceof Active && lastRun[i] != Values[i]) {
                ((Active) (hardware.get(i))).setHardware(Values[i]);
            }
            //instanceof and typecasting allows for sensors to not include setHardware.
        }

        lastRun = Values;
    }

    public void Stop(){
        stop = true;
    }
}
