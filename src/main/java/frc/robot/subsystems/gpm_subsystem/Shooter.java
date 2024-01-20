package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shooter extends CommandBase {

    private final WPI_TalonFX shooterMotor;
    private final double shooterSpeed;
    private final double duration;

    public Shooter(int motorId, double shooterSpeed, double duration) {
        this.shooterSpeed = shooterSpeed;
        this.duration = duration;

        // Initialize shooter subsystem
        shooterMotor = new WPI_TalonFX(motorId);
        configureShooterMotor();
        addRequirements(this);

        // Configure additional requirements if needed
    }

    private void configureShooterMotor() {
        // Configure motor settings, such as inversion, sensor phase, etc.
        // For example:
        shooterMotor.setInverted(false);
        shooterMotor.setSensorPhase(false);
        // Add more configuration as needed
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
        setTimeout(duration);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // No additional execution logic needed
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isTimedOut();
    }
}


/*  for scheduling and running (in robot container or elsewhere) for example:

Shooter shooterRun = new Shooter(1, 0.5, 0.6);
shooter.schedule();

*/
