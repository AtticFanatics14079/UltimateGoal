package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DDistanceSensor implements Sensor, DriveObject, DistanceSensor {

    private DistanceSensor sensor;
    private int partNum;

    private Thread t = null;

    public volatile boolean gettingInput = true;

    public DDistanceSensor(HardwareMap hwMap, String objectName) {
        sensor = hwMap.get(DistanceSensor.class, objectName);
        this.partNum = hardware.size();

        hardware.add(this);
    }

    @Override
    public int getPartNum() {
        return partNum;
    }

    @Override
    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    @Override
    public double[] getHardware() {
        return new double[]{sensor.getDistance(DistanceUnit.INCH)};
    }

    @Override
    public void endThreads() {
        //if(t != null && t.isAlive()) t.stop();
        //Extra cycle should take care of this.
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        switch(unit) {
            case INCH: return get()[0];
            case CM: return 2.54 * get()[0];
            case MM: return 25.4 * get()[0];
            case METER: return 0.0254 * get()[0];
        }
        return Double.MAX_VALUE;
    }

    public void pingSensor() {
        Sequence pingSensor = new Sequence(() -> {
            gettingInput = true;
            vals.waitForCycle();
            gettingInput = false;
            return null;
        });
        if(t != null && t.isAlive()) return;
        t = new Thread(pingSensor);
        t.start();
    }

    public void retrievingHardware(boolean retrieving) {
        gettingInput = retrieving;
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
