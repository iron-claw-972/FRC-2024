package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import lib.drivers.LazyTalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.globalConst;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.ConversionUtils;
import frc.robot.util.LogManager;
import lib.CTREModuleState;

public class Module extends SubsystemBase {
    private final ShuffleboardTab swerveTab;
    private final ModuleType type;

    private final Rotation2d angleOffset;

    private final LazyTalonFX angleMotor;
    private final LazyTalonFX driveMotor;
    private final WPI_CANCoder CANcoder;
    private SwerveModuleState desiredState;

    private boolean stateDeadband;

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
        angleMotor = new LazyTalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new LazyTalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
        configDriveMotor();

        setDesiredState(new SwerveModuleState(0, getAngle()), false);

        setupShuffleboard();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = optimizeStates ? CTREModuleState.optimize(desiredState, getState().angle) : desiredState;
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
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
        if (globalConst.DO_LOGGING) {
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
        if (globalConst.DO_LOGGING) {
            double position = ConversionUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(),
                                                              DriveConstants.kAngleGearRatio);
            LogManager.addDouble("Swerve/Modules/SteerPosition/" + type.name(),
                                 position
                                );
            LogManager.addDouble("Swerve/Modules/SteerPositionError/" + type.name(),
                                 position - desiredState.angle.getDegrees()
                                );
            LogManager.addDouble("Swerve/Modules/SteerVelocity/" + type.name(),
                                 ConversionUtils.falconToDegrees(angleMotor.getSelectedSensorVelocity(),
                                                                 DriveConstants.kAngleGearRatio)
                                );
            LogManager.addDouble("Swerve/Modules/SteerVoltage/" + type.name(),
                                 angleMotor.getMotorOutputVoltage()
                                );
            LogManager.addDouble("Swerve/Modules/SteerCurrent/" + type.name(),
                                 angleMotor.getStatorCurrent()
                                );
        }
    }

    public void enableStateDeadband(boolean enabled) {
        stateDeadband = enabled;
        LogManager.addBoolean("Swerve/Modules/StateDeadband/" + type.name(), enabled);
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
        LogManager.addBoolean("Swerve/Modules/Optimized/" + type.name(), enable);
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                ConversionUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConstants.kAngleGearRatio));
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
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
        angleMotor.configVoltageCompSaturation(globalConst.ROBOT_VOLTAGE);
        angleMotor.enableVoltageCompensation(true);
        resetToAbsolute();
    }

    public void setDriveCharacterizationVoltage(double voltage) {
        angleMotor.set(ControlMode.Position, ConversionUtils.degreesToFalcon(0, DriveConstants.kAngleGearRatio));
        driveMotor.set(ControlMode.PercentOutput, voltage / globalConst.ROBOT_VOLTAGE);
        if (globalConst.DO_LOGGING) {
            LogManager.addDouble("Swerve/Modules/DriveCharacterizationVoltage/" + type.name(),
                                 voltage
                                );
        }
    }

    public void setAngleCharacterizationVoltage(double voltage) {
        angleMotor.set(ControlMode.PercentOutput, voltage / globalConst.ROBOT_VOLTAGE);
        // Set the drive motor to just enough to overcome static friction
        driveMotor.set(ControlMode.PercentOutput, 1.1 * DriveConstants.DRIVE_KS);
        if (globalConst.DO_LOGGING) {
            LogManager.addDouble("Swerve/Modules/AngleCharacterizationVoltage/" + type.name(),
                                 voltage
                                );
        }
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
        driveMotor.configVoltageCompSaturation(globalConst.ROBOT_VOLTAGE);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConstants.kWheelCircumference,
                                            DriveConstants.kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(driveMotor.getSelectedSensorPosition(), DriveConstants.kWheelCircumference,
                                               DriveConstants.kDriveGearRatio),
                getAngle());
    }

    private void setupShuffleboard() {
        if (RobotBase.isReal() && globalConst.USE_TELEMETRY) {
            swerveTab.addDouble(type.name() + " CANcoder Angle (deg)", getCANcoder()::getDegrees);
            swerveTab.addDouble(type.name() + " FX Angle (deg)", getPosition().angle::getDegrees);
            swerveTab.addDouble(type.name() + " Velocity (m/s)", () -> getState().speedMetersPerSecond);
            swerveTab.addDouble(type.name() + " Desired Velocity (m/s)", () -> getDesiredState().speedMetersPerSecond);
            swerveTab.addDouble(type.name() + " Desired Angle (deg)", () -> getDesiredState().angle.getDegrees());
            swerveTab.addBoolean(type.name() + " Jitter prevention enabled", () -> stateDeadband);
            swerveTab.addDouble(type.name() + " Drive Current (A)", driveMotor::getSupplyCurrent);
            swerveTab.addDouble(type.name() + " Angle Current (A)", angleMotor::getSupplyCurrent);
        }
    }

    @Override
    public void periodic() {

    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
    }

    public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
    }

    public double getDriveVelocityError() {
        return getDesiredState().speedMetersPerSecond - getState().speedMetersPerSecond;
    }

    public double getDriveFeedForwardKV() {
        return DriveConstants.DRIVE_KV;
    }

    public double getDriveFeedForwardKS() {
        return DriveConstants.DRIVE_KS;
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void setDriveVoltage(double volts) {
        //with voltage compensation enabled do not use setVoltage
    }

    public void setSteerVoltage(double voltage) {
        //with voltage compensation enabled do not use setVoltage
    }

    public void setDriveFeedForwardValues(double kS, double kV) {
    }

    public double getSteerFeedForwardKV() {
        return 0;
    }

    public double getSteerFeedForwardKS() {
        return 0;
    }

    public void setAngle(Rotation2d rotation2d) {
    }
}