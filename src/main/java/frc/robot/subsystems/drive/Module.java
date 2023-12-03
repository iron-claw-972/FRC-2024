package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GlobalConst;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.ConversionUtils;
import lib.CTREModuleState;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.LogManager;

/**
 * Swerve module for drivetrain
 */
public class Module extends SubsystemBase {

    private double currentSteerPositionRad = 0;
    private double currentDrivePositionMeters = 0;
    private double currentSpeed = 0;

    protected boolean stateDeadband = true;
    private ShuffleboardTab swerveTab;
    
    private final ModuleType type;
    
    // Motor ticks
    private final Rotation2d angleOffset;

    private final WPI_TalonFX angleMotor;
    private final WPI_TalonFX driveMotor;
    private final WPI_CANCoder CANcoder;
    private SwerveModuleState desiredState;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA);

    private boolean optimizeStates = true;
    public Module(ModuleConstants moduleConstants, ShuffleboardTab swerveTab) {
        this.swerveTab = swerveTab;

        type = moduleConstants.getType();

        angleOffset = new Rotation2d(moduleConstants.getSteerOffset());

        stateDeadband = true;

        /* Angle Encoder Config */
        CANcoder = new WPI_CANCoder(moduleConstants.getEncoderPort(), DriveConstants.kSteerEncoderCAN);
        configCANcoder();

        /* Angle Motor Config */
        angleMotor = new WPI_TalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new WPI_TalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
        configDriveMotor();

        setDesiredState(new SwerveModuleState(0, getAngle()), false);

    }

    /**
     * Updates the simulation
     */
    @Override
    public void periodic() {
        currentDrivePositionMeters += currentSpeed * GlobalConst.LOOP_TIME;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @param isOpenLoop   whether to use closed/open loop control for drive velocity
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(RobotBase.isReal())  {
                /*
            * This is a custom optimize function, since default WPILib optimize assumes
            * continuous controller which CTRE and Rev onboard is not
            */
            this.desiredState = optimizeStates ? CTREModuleState.optimize(desiredState, getState().angle) : desiredState;
            setAngle(desiredState);
            setSpeed(desiredState, isOpenLoop);
        }
        else {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            currentSpeed = 0;
            return;
        }
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = CTREModuleState.optimize(desiredState, new Rotation2d(currentSteerPositionRad));

        currentSpeed = desiredState.speedMetersPerSecond;
        currentSteerPositionRad = desiredState.angle.getRadians();
        }
    }

    /**
     * Gets the simulated angle of the module.
     */
    public Rotation2d getAngle() {
        if(RobotBase.isReal())  {
            return Rotation2d.fromDegrees(
                ConversionUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConstants.kAngleGearRatio));
        }
        return new Rotation2d(currentSteerPositionRad);
        
    }

    // TODO: Comment
    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

 

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = ConversionUtils.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.kWheelCircumference,
                                                          DriveConstants.kDriveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                           feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        if (GlobalConst.DO_LOGGING) {
            double motorSpeed = ConversionUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConstants.kWheelCircumference,
                                                            DriveConstants.kDriveGearRatio);
            LogManager.addDouble("Swerve/Modules/DriveSpeed/" + type.name(),
                                 motorSpeed
                                );
            LogManager.addDouble("Swerve/Modules/DriveSpeedError/" + type.name(),
                                 motorSpeed - desiredState.speedMetersPerSecond
                                );
            LogManager.addDouble("Swerve/Modules/DriveVoltage/" + type.name(),
                                 driveMotor.getMotorOutputVoltage()
                                );
            LogManager.addDouble("Swerve/Modules/DriveCurrent/" + type.name(),
                                 driveMotor.getStatorCurrent()
                                );
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if desired speed < 1%. Prevents Jittering.
        if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))) {
            stop();
            return;
        }
        angleMotor.set(ControlMode.Position, ConversionUtils.degreesToFalcon(desiredState.angle.getDegrees(), DriveConstants.kAngleGearRatio));
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        // Sensor ticks
        // TODO: Convert sensor ticks in driveconstants to radians
        double absolutePosition = ConversionUtils.degreesToFalcon(getCANcoder().getDegrees() - angleOffset.getDegrees(),
                                                                  DriveConstants.kAngleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);                                                         
    }

    private void configCANcoder() {
        CANcoder.configFactoryDefault();
        CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        CANcoder.configSensorDirection(DriveConstants.kModuleConstants.canCoderInvert);
        CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        CANcoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                DriveConstants.kAngleEnableCurrentLimit,
                DriveConstants.kAngleContinuousCurrentLimit,
                DriveConstants.kAnglePeakCurrentLimit,
                DriveConstants.kAnglePeakCurrentDuration
        ));
        angleMotor.config_kP(0, DriveConstants.kModuleConstants.angleKP);
        angleMotor.config_kI(0, DriveConstants.kModuleConstants.angleKI);
        angleMotor.config_kD(0, DriveConstants.kModuleConstants.angleKD);
        angleMotor.config_kF(0, DriveConstants.kModuleConstants.angleKF);
        angleMotor.setInverted(DriveConstants.kAngleMotorInvert);
        angleMotor.setNeutralMode(DriveConstants.kAngleNeutralMode);
        angleMotor.configVoltageCompSaturation(GlobalConst.ROBOT_VOLTAGE);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setSelectedSensorPosition(0);
        resetToAbsolute();
    }

    public double getSteerVelocity() {
        return ConversionUtils.falconToRPM(angleMotor.getSelectedSensorVelocity(), DriveConstants.kAngleGearRatio) * 2 * Math.PI / 60;
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                DriveConstants.kDriveEnableCurrentLimit,
                DriveConstants.kDriveContinuousCurrentLimit,
                DriveConstants.kDrivePeakCurrentLimit,
                DriveConstants.kDrivePeakCurrentDuration
        ));
        driveMotor.config_kP(0, DriveConstants.kDriveP);
        driveMotor.config_kI(0, DriveConstants.kDriveI);
        driveMotor.config_kD(0, DriveConstants.kDriveD);
        driveMotor.config_kF(0, DriveConstants.kDriveF);
        driveMotor.configOpenloopRamp(DriveConstants.kOpenLoopRamp);
        driveMotor.configClosedloopRamp(DriveConstants.kClosedLoopRamp);
        driveMotor.setInverted(DriveConstants.kDriveMotorInvert);
        driveMotor.setNeutralMode(DriveConstants.kDriveNeutralMode);
        driveMotor.configVoltageCompSaturation(GlobalConst.ROBOT_VOLTAGE);
        driveMotor.enableVoltageCompensation(true);
    }

    public SwerveModuleState getState() {
        if(RobotBase.isReal())  {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConstants.kWheelCircumference,
                                            DriveConstants.kDriveGearRatio),
                getAngle());
        }
        return new SwerveModuleState(
                currentSpeed,
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        if(RobotBase.isReal())  {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(driveMotor.getSelectedSensorPosition(), DriveConstants.kWheelCircumference,
                                               DriveConstants.kDriveGearRatio),
                getAngle());
        }
        return new SwerveModulePosition(
                currentDrivePositionMeters,
                new Rotation2d(currentSteerPositionRad)
        );
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }


    public double getDriveVelocityError() {
        return getDesiredState().speedMetersPerSecond - getState().speedMetersPerSecond;
    }

    public void stop() {
        if(RobotBase.isReal())  {
        driveMotor.set(0);
        angleMotor.set(0);
        }
        else {
            currentSpeed = 0;
        }
    }
}