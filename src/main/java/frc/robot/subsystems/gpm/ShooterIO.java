package frc.robot.subsystems.gpm;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    // Logging all inputs
    @AutoLog
  	class ShooterIOInputs {
    	public double leftFlywheelRPM = 0.0;
    	public double leftFlywheelVolts = 0.0;

    	public double rightFlywheelRPM = 0.0;
    	public double rightFlywheelVolts = 0.0;
	}
    // Update Inputs
    default void updateInputs(ShooterIOInputsAutoLogged shooterInputs) {}
}
