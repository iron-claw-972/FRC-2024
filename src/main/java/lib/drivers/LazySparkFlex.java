package lib.drivers;

import com.revrobotics.CANSparkFlex;
public class LazySparkFlex extends CANSparkFlex {

    protected double mLastSet = Double.NaN;
    public LazySparkFlex(int deviceId, MotorType type) {
        super(deviceId,type);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(double speed) {
        if (speed != mLastSet) {
            mLastSet = speed;
            super.set(speed);
        }
    }

}
