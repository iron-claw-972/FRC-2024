package lib.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends WPI_TalonFX {
    protected double mLastSet = Double.NaN;

    public LazyTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(double value) {
        if (value != mLastSet) {
            mLastSet = value;
            super.set(value);
        }
    }
}