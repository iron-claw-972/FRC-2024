package lib.drivers;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This class is a thin wrapper around the TalonFX that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double mLastSet = Double.NaN;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public LazyTalonFX(int deviceNumber,String canString) {
        super(deviceNumber,canString);
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