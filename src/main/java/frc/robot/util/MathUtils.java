package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

/**
 * Utility class for useful functions.
 */
public class MathUtils {

    /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be linear from
     * (deadband, 0) to (1,1)
     *
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    /**
     * Deadbands an input to [-1, -OIConstants.DEADBAND], [OIConstants.DEADBAND, 1], rescaling inputs to be linear from
     * (OIConstants.DEADBAND, 0) to (1,1)
     *
     * @param input The input value to rescale
     * @return the input rescaled and to fit [-1, -DEADBAND], [DEADBAND, 1]
     */
    public static double deadband(double input) {
        return deadband(input, Constants.DEADBAND);
    }

    /**
     * An exponential function that maintains positive or negative sign.
     *
     * @param exponent the power to raise the base to
     * @param base     the base which will be raised to the power
     * @return base to the power of exponent, maintaining sign of base
     */
    public static double expoMS(double base, double exponent) {
        // weird stuff will happen if you don't put a number > 0 for controller inputs
        double finVal = Math.pow(Math.abs(base), exponent);
        if (base < 0) {
            finVal *= -1;
        }
        return finVal;
    }

    /**
     * Calculates a points distance from the origin
     *
     * @param x X coordinate
     * @param y Y coordinate
     * @return distance from (0,0)
     */
    public static double calculateHypotenuse(double x, double y) {
        return Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5);
    }

    /**
     * Calculates Midpoint of two numbers on modulus number line
     *
     * @param num1       first number
     * @param num2       second number
     * @param lowerBound lower bound of modulus number line
     * @param upperBound upper bound of modulus number line
     * @return midpoint of 2 numbers on modulus number line
     */
    public static double modulusMidpoint(double num1, double num2, double lowerBound, double upperBound) {
        num1 = MathUtil.inputModulus(num1, lowerBound, upperBound);
        num2 = MathUtil.inputModulus(num2, lowerBound, upperBound);
        if (Math.abs(num1 - num2) > (upperBound - lowerBound) / 2) {
            return MathUtil.inputModulus((num1 + num2) / 2 + (upperBound - lowerBound) / 2, lowerBound, upperBound);
        }
        return (num1 + num2) / 2;
    }

    /**
     * Calls {@link #mean(double...)}.
     *
     * @param data the list of data to find the mean of
     * @return the mean of the data
     */
    public static double mean(List<Double> data) {
        return mean(doubleListToArray(data));
    }

    /**
     * Finds the mean of the provided array of doubles
     *
     * @param data an array of doubles
     * @return the mean of the data
     */
    public static double mean(double... data) {
        double mean = 0;
        for (double datum : data) {
            mean += datum;
        }
        mean /= data.length;
        return mean;
    }

    /**
     * Calls {@link #stdDev(double...)}.
     *
     * @param data the list of data to find the standard deviation of
     * @return the standard deviation of the data
     */
    public static double stdDev(List<Double> data) {
        return stdDev(doubleListToArray(data));
    }

    /**
     * Finds the standard deviation of the provided array of doubles
     *
     * @param data an array of doubles
     * @return the standard deviation of the data
     */
    public static double stdDev(double... data) {
        if (data.length == 0 || data.length == 1) return 0;

        double mean = mean(data);

        double total = 0;
        for (double datum : data) {
            total += Math.pow(datum - mean, 2);
        }
        return Math.sqrt(total / (data.length - 1));
    }

	/**
	 * Converts a firmware version to a number that getFirmware likes.
	 * <p>
	 * Versions are typically of the form major.minor.patch (eg: 22.1.1 to 5633).
	 * 
	 * @param major the major version number
	 * @param minor the minor version number
	 * @param patch the patch number
	 * @return the converted number
	 */
	public static int talonEncodeFirmwareVersion(int major, int minor, int patch) {
		return (major << 8) + minor;
	}

	/**
	 * Converts a firmware version to a number that getFirmwareVersion likes.
	 * 
	 * @param major the major version number
	 * @param minor the minor version number
	 * @param build_h the build H
	 * @param build_l the build L
	 * @return the converted number
	 */
	public static int revEncodeFirmwareVersion(int major, int minor, int build_h, int build_l) {
		return (major << 24) + (minor << 16) + (build_h << 8) + build_l;
	}

    private static double[] doubleListToArray(List<Double> arrayList) {
        return arrayList.stream().mapToDouble(Double::doubleValue).toArray();
    }

}
