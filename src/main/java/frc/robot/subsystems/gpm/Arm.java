package frc.robot.subsystems.gpm;

// ghost arm.
public class Arm {
    private double angle;
    public Arm() {
        this.angle = .8; // pi/4
    }
    public double getAngle() { return angle;}
    public void setAngle(double ang) {angle = ang;}
}
