package lib.ctre_shims;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Class to control a simulated encoder.
 */
public class TalonEncoderSim {
    private TalonEncoder m_encoder;

    private TalonSRXSimCollection simCollectionSRX = null;
    private TalonFXSimCollection simCollectionFX = null;

    /**
     * Constructs from an Encoder object.
     *
     * @param encoder Encoder to simulate
     */
    public TalonEncoderSim(TalonEncoder encoder) {
        m_encoder = encoder;

        if (encoder.getMotor() instanceof TalonSRX) {
            simCollectionSRX = new TalonSRXSimCollection(encoder.getMotor());
        } else if (encoder.getMotor() instanceof TalonFX) {
            simCollectionFX = new TalonFXSimCollection(encoder.getMotor());
        } else {
            throw new IllegalStateException(
                    "Motor type " + encoder.getMotor().getClass().getName() + " is unsupported.");
        }
    }

    /**
     * Read the count of the encoder.
     *
     * @return the count
     */
    public int getCount() {
        return m_encoder.get();
    }

    /**
     * Change the count of the encoder.
     *
     * @param count the new count
     */
    public void setCount(int count) {
        if (simCollectionSRX != null) {
            FeedbackDevice selected = m_encoder.getSelectedFeedbackSensor();

            switch (selected) {
                case QuadEncoder:
                case CTRE_MagEncoder_Relative:
                    simCollectionSRX.setQuadratureRawPosition(count);
                    break;

                case Analog:
                    simCollectionSRX.setAnalogPosition(count);
                    break;

                default:
                    throw new IllegalStateException(
                            "Selected feedback sensor is not supported: " + selected.name());
            }
        } else if (simCollectionFX != null) {
            simCollectionFX.setIntegratedSensorRawPosition(count);
        } else {
            // This should have errored already in the constructor.
            assert false;
        }
    }

    /**
     * Read the period of the encoder.
     *
     * @return the encoder period
     */
    public double getPeriod() {
        return m_encoder.getRate();
    }

    /**
     * Change the encoder period.
     *
     * @param period the new period
     */
    public void setPeriod(double period) {
        // seconds -> distance/second
        // distancePerPulse (distance) / period (seconds) = distance / second
        setRate(m_encoder.getDistancePerPulse() / period);
    }

    // These are no-op in WPILib
    // boolean getReset();
    // void setReset();

    /**
     * Get the direction of the encoder.
     *
     * @return the direction of the encoder
     */
    public boolean getDirection() {
        return m_encoder.getDirection();
    }

    /**
     * Get the samples-to-average value.
     *
     * <p>See {@link TalonEncoder#getSamplesToAverage()}.
     *
     * @return the samples-to-average value
     */
    public int getSamplesToAverage() {
        return m_encoder.getSamplesToAverage();
    }

    /**
     * Set the samples-to-average value.
     *
     * <p>See {@link TalonEncoder#setSamplesToAverage(int)}.
     *
     * @param samplesToAverage the new value
     */
    public void setSamplesToAverage(int samplesToAverage) {
        m_encoder.setSamplesToAverage(samplesToAverage);
    }

    /**
     * Change the encoder distance.
     *
     * @param distance the new distance
     */
    public void setDistance(double distance) {
        setCount((int) (distance / m_encoder.getDistancePerPulse()));
    }

    /**
     * Read the distance of the encoder.
     *
     * @return the encoder distance
     */
    public double getDistance() {
        return m_encoder.getDistance();
    }

    /**
     * Change the rate of the encoder.
     *
     * @param rate the new rate
     */
    public void setRate(double rate) {
        int rateInNativeUnits = (int) (rate / (10 * m_encoder.getDistancePerPulse()));

        if (simCollectionSRX != null) {
            FeedbackDevice selected = m_encoder.getSelectedFeedbackSensor();

            switch (selected) {
                case QuadEncoder:
                case CTRE_MagEncoder_Relative:
                    simCollectionSRX.setQuadratureVelocity(rateInNativeUnits);
                    break;

                case Analog:
                    simCollectionSRX.setAnalogVelocity(rateInNativeUnits);
                    break;

                default:
                    throw new IllegalStateException(
                            "Selected feedback sensor is not supported: " + selected.name());
            }
        } else if (simCollectionFX != null) {
            simCollectionFX.setIntegratedSensorVelocity(rateInNativeUnits);
        } else {
            // This should have errored already in the constructor.
            assert false;
        }
    }

    /**
     * Get the rate of the encoder.
     *
     * @return the rate of change
     */
    public double getRate() {
        return m_encoder.getRate();
    }

    /**
     * Resets all simulation data for this encoder.
     */
    public void resetData() {
        setDistance(0);
        setRate(0);
    }
}
