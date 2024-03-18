package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerPanel extends SubsystemBase {
	private static final PowerDistribution PDH = new PowerDistribution();
	private static PDPSim PDHSim; // not sure if this is only CTRE or what
	private double voltsBattery = 12.6;
	// assume the battery resistance is about 25 milohms + some wire resistance
	private static double ohmsResistance = 0.030;
	
	public PowerPanel() {
		if (RobotBase.isSimulation()) {
			PDHSim = new PDPSim(PDH);

			// TODO: find actual values for things like Beelink
			PDHSim.setCurrent(18, 12.4); //this is an example
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("PDH Current (Amps)", PDH.getTotalCurrent());

		// simulate the voltage on the battery
		voltsBattery = 12.6 - PDH.getTotalCurrent() * ohmsResistance;
	}

	@Override
	public void simulationPeriodic() {

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
	 * This method is used by simulators to set the channel current
	 * @param channel PDH port/channel
	 * @param current Channel current in amperes.
	 */
	public void setCurrent(int channel, double current) {
		PDHSim.setCurrent(channel, current);
	}

}

