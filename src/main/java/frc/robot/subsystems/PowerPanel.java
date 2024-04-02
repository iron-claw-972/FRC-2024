package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Power Distribution Panel/Hub.
 * <p>
 * This is an information subsystem that is used by other subsystems or commands.
 * It should not be a Requirement for any subsystem.
 * <p>
 * See https://docs.wpilib.org/en/stable/docs/software/can-devices/power-distribution-module.html
 * <p>
 * See https://docs.google.com/spreadsheets/d/1UiHZFYeZiHPAPIu39uRrskQuQYfvJ03UjLeQVq--Mzg/edit#gid=0 for PDH assignments.
 */
public class PowerPanel extends SubsystemBase {
	// We use the REV Power Distribution Hub (PDH) at CAN Id 1
	private static final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
	// PDP/PDH simulation resources
	private static PDPSim PDHSim;
	/** The simulated battery voltage */
	private double voltsBattery = 12.6;
	// assume the battery resistance is about 25 milohms + some wire resistance
	private static double ohmsResistance = 0.030;
	
	public PowerPanel() {
		// if we are simulating
		if (RobotBase.isSimulation()) {
			PDHSim = new PDPSim(PDH);

			// TODO: find actual values for things like Beelink
			// this is just an example...
			PDHSim.setCurrent(18, 12.4);
		}
	}

	@Override
	public void periodic() {
		// put the current draw on the SmartDashboard
		// TODO: this call may be slow....
		SmartDashboard.putNumber("PDH Current", PDH.getTotalCurrent());

		// TODO: put the Energy draw on the SmartDashboard

		// simulate the voltage on the battery
		voltsBattery = 12.6 - PDH.getTotalCurrent() * ohmsResistance;
	}

	@Override
	public void simulationPeriodic() {

	}

	/**
	 * Return the battery voltage.
	 * Can be used in simulations.
	 * @return battery voltage
	 */
	public double getSimulatedBatteryVoltage() {
		return voltsBattery;
	}

	/**
	 * Return the current drawn by the PDH channel.
	 * @param channel
	 * @return current in amperes
	 */
	public double getCurrent(int channel) {
		return PDH.getCurrent(channel);
	}

	/**
	 * This method is used by simulators to set the channel current.
	 * <p>
	 * This method should only be called when simulating.
	 * If it is called on a real robot, it will raise a null pointer exception.
	 * @param channel PDH port/channel
	 * @param current Channel current in amperes.
	 */
	public void setCurrent(int channel, double current) {
		PDHSim.setCurrent(channel, current);
	}

}
