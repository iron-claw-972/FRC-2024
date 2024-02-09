// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants;
import frc.robot.constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.Setpoints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

/* 
 * Arm - Subsystem to control all Arm motion using a Trapezoidal Profiled PID controller
 * 
 * For more details on how this works, see:
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
 *
 */
public class Arm extends  {

    /* Creates a new ArmSubsystem */
    private TalonFX m_armLeader = new TalonFX(CanConstants.ID_ArmLeader);
    private TalonFX m_armFollower = new TalonFX(CanConstants.ID_ArmFollower);

    /* Set up to control the motors by Voltage */
    private VoltageOut m_VoltageOutput = new VoltageOut(0.0);

    /* Neutral output control for disabling the Arm */
    private final NeutralOut m_neutral = new NeutralOut();

    /* Set up the REV Through-Bore encoder as a DutyCycle (Absolute) encoder */
    private DutyCycleEncoder m_encoder = new DutyCycleEncoder(DIOConstants.kArmAbsEncoder);

    /* Feed Forward object to assist Profile Controller */
    private ArmFeedforward m_feedforward = new ArmFeedforward(
            constants.ArmConstants.kS,
            constants.ArmConstants.kG,
            constants.ArmConstants.kV,
            constants.ArmConstants.kA);

    /* Working (current) setpoint */
    private double m_armSetpoint;

    /* Working (current) tolerance */
    private double m_tolerance;

    /*
     * Constructor
     */
    public Arm() {

        /* Create the Trapezoidal motion profile controller */
        super(
                new ProfiledPIDController(
                        ArmConstants.kP,
                        ArmConstants.kI,
                        ArmConstants.kD,
                        new TrapezoidProfile.Constraints(ArmConstants.kArm_Cruise, ArmConstants.kArm_Acceleration)));

        // Start arm at rest in STOWED position
        updateArmSetpoint(RobotConstants.STOWED);

        // Config Duty Cycle Range for the encoders
        m_encoder.setDutyCycleRange(ArmConstants.kDuty_Cycle_Min, ArmConstants.kDuty_Cycle_Max);

        // Config Motors
        var leadConfiguration = new TalonFXConfiguration();
        var followerConfiguration = new TalonFXConfiguration();

        // Set the output mode to brake
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set the motor's Neutral Deadband
        leadConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.kNeutral_Deadband;
        followerConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.kNeutral_Deadband;

        /* Set the turning direction */
        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLeader.getConfigurator().apply(leadConfiguration);
        m_armFollower.getConfigurator().apply(followerConfiguration);
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), false));

    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        // Make sure the parent controller gets to do its own updates
        super.periodic();

        // Validate current encoder reading; stop motors if out of range
        /*
         * double armPos = getArmJointDegrees();
         * if (!m_encoder.isConnected() || ( armPos < 0.0 || armPos >= 360.0)) {
         * System.out.println("Arm Encoder error reported in periodic().");
         * neutralOutput();
         * }
         */
        // Display useful info on the SmartDashboard
        SmartDashboard.putData("Arm Controller", m_armLeader);
        SmartDashboard.putBoolean("Arm Joint at Setpoint?", isArmJointAtSetpoint());
        SmartDashboard.putNumber("Raw Arm Encoder ", getJointPosAbsolute());
        SmartDashboard.putNumber("Arm Current Angle", getArmJointDegrees());

        if (Constants.RobotConstants.kIsTuningMode) {
            SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getJointPosAbsolute()));
            SmartDashboard.putNumber("Arm Joint Error", getArmJointError());
            SmartDashboard.putNumber("Arm Joint Setpoint", m_armSetpoint);
        }
    }

    /**
     * Consumes the output from the ProfiledPIDController.
     * 
     * The PIDSubsystem will automatically call this method from its periodic()
     * block, and pass it the computed output of the control loop.
     * 
     * @param output   the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController
     */
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {

        // Correct the passed-in current setpoint before calculating the feedforward
        double correctedPosition = correctArmJointRadiansForFeedFwd(setpoint.position);

        // Calculate the feedforward using the corrected setpoint
        double feedforward = m_feedforward.calculate(correctedPosition, setpoint.velocity);

        if (Constants.RobotConstants.kIsTuningMode) {
            SmartDashboard.putNumber("Arm corrected FF position", correctedPosition);
            SmartDashboard.putNumber("Arm PID output", output);
            SmartDashboard.putNumber("Arm Feed Forward Output", feedforward);
        }

        // Add the feedforward to the PID output to get the motor output
        m_armLeader.setControl(m_VoltageOutput.withOutput(output + feedforward));
    }

    /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * The PIDSubsystem will automatically call this method from its periodic()
     * block, and pass the returned value to the control loop.
     * 
     * @return the measurement of the process variable, in this case, the Arm angle,
     *         in radians corrected to 0.0 at the STOWED position
     */
    @Override
    public double getMeasurement() {
        return getArmJointRadians();
    }

    /**
     * Update the PID controller's current Arm setpoint in radians
     * 
     * @param setpointDegrees - the desired position in degrees
     */
    public void updateArmSetpoint(Setpoints setpoints) {

        // Convert degrees to radians and set the profile goal
        m_armSetpoint = setpoints.arm;
        m_tolerance = setpoints.tolerance;
        setGoal(degreesToRadians(setpoints.arm));
    }

    /** Override the enable() method so we can set the goal to the current position
     * 
     *  The super method resets the controller and sets its current setpoint to the 
     *  current position, but it does not reset the goal, which will cause the Arm
     *  to jump from the current position to the old goal. 
     */
    @Override
    public void enable() {
        super.enable();
        m_armSetpoint = getArmJointDegrees(); 
        setGoal(getArmJointRadians());
    }

    // Get the current Arm Joint position error (in degrees)
    public double getArmJointError() {
        return Math.abs(m_armSetpoint - getArmJointDegrees());
    }

    // Check if Arm is at the setpoint (or within tolerance)
    public boolean isArmJointAtSetpoint() {
        return getArmJointError() < m_tolerance;
    }

    // Drive the Arm directly by providing a supply voltage value
    public void setArmVoltage(double voltage) {
        m_armLeader.setControl(m_VoltageOutput.withOutput(voltage));
    }

    // Set the lead motor to the Neutral state (no output)
    public void neutralOutput() {
        m_armLeader.setControl(m_neutral);
    }

    // Returns the current encoder absolute value in DutyCycle units (~0 -> ~1)
    public double getJointPosAbsolute() {
        return m_encoder.getAbsolutePosition();
    }

    // Converts DutyCycle units to Degrees
    public double dutyCycleToDegrees(double dutyCyclePos) {
        return dutyCyclePos * 360;
    }

    // Converts the current encoder reading to Degrees, and corrects relative to a
    // STOWED position of zero.
    public double getArmJointDegrees() {
        return dutyCycleToDegrees(getJointPosAbsolute()) - ArmConstants.kARM_STARTING_OFFSET;
    }

    // Converts DutyCycle units to Radians
    public double dutyCycleToRadians(double dutyCyclePos) {
        return dutyCyclePos * 2.0 * Math.PI;
    }

    // Converts degrees to Radians
    public double degreesToRadians(double degrees) {
        return (degrees * Math.PI) / 180.0;
    }

    // Converts the current encoder reading to Degrees, and corrects relative to a
    // STOWED position of zero.
    public double getArmJointRadians() {
        return dutyCycleToRadians(getJointPosAbsolute()) - degreesToRadians(ArmConstants.kARM_STARTING_OFFSET);
    }

    // Takes a position in radians relative to STOWED, and corrects it to be
    // relative to a HORIZONTAL position of zero.
    // This is used for Feedforward only, where we account for gravity using a
    // cosine function
    public double correctArmJointRadiansForFeedFwd(double position) {
        return position - degreesToRadians(ArmConstants.kARM_HORIZONTAL_OFFSET - ArmConstants.kARM_STARTING_OFFSET);
    }

    /*
     * Command Factories
     */

    // To position for Intake, move Arm to INTAKE position
    public Command prepareForIntakeCommand() {
        return new RunCommand(()-> this.updateArmSetpoint(RobotConstants.INTAKE), this)
            .until(()->this.isArmJointAtSetpoint());
    }

}
