package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Open some DIO inputs and display them on the SmartDashboard.
 */
public class DIOCheck extends SubsystemBase {
    DigitalInput[] adio = new DigitalInput[12];
    DIOSim[] adioSim = new DIOSim[12];
    String[] label = new String[12];
    int counterSim = 0;

    public DIOCheck() {
        for (int i = 0; i < adio.length; i++) {
            adio[i] = new DigitalInput(i);
            label[i] = "DIO " + i;

            if (RobotBase.isSimulation()) {
                adioSim[i] = new DIOSim(adio[i]);
                adioSim[i].setValue((i&1) != 0);
            }
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < adio.length; i++) {
            SmartDashboard.putBoolean(label[i], adio[i].get());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (counterSim++ > 50) {
            counterSim = 0;

            for (int i = 0; i < adio.length; i++) {
                adioSim[i].setValue(!adio[i].get());
            }
        }
    }
}
