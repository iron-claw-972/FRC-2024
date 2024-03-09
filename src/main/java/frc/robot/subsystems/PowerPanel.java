package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerPanel extends SubsystemBase {
	private static final PowerDistribution PDH = new PowerDistribution();
	private static PDPSim PDHSim; // not sure if this is only CTRE or what
	
	private PowerPanel() {
		if (RobotBase.isSimulation()) {
			PDHSim = new PDPSim(PDH);

			// TODO: find actual values for things like Beelink
			PDHSim.setCurrent(18, 12.4); //this is an example
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("PDH Current (Amps)", PDH.getTotalCurrent());
	}

	@Override
	public void simulationPeriodic() {
	}
}

