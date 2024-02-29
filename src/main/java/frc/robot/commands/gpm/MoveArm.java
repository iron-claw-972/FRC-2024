package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Arm;

public class MoveArm extends Command{

    private final Arm m_arm;
    private final double m_setpoint;
    
    public MoveArm(Arm arm, double setpoint) {
        m_arm = arm;
        m_setpoint = setpoint;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setAngle(m_setpoint);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }

    @Override
    public void end(boolean interupted) {

    }


}
