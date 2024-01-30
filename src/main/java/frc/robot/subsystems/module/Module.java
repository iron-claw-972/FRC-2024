package frc.robot.subsystems.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.ConversionUtils;
import frc.robot.util.LogManager;
import lib.CTREModuleState;

public class Module extends SubsystemBase {
    private final ModuleType type;
    
    // Motor ticks
    private final double angleOffset;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANcoder CANcoder;
    private SwerveModuleState desiredState;

    protected boolean stateDeadband = true;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA);
    
    final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    
    private boolean optimizeStates = true;

    ModuleConstants moduleConstants;

    public Module(ModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;

        type = moduleConstants.getType();

        //angleOffset = new Rotation2d(constants.getSteerOffset());
        angleOffset = moduleConstants.getSteerOffset();

        /* Angle Encoder Config */
        CANcoder = new CANcoder(moduleConstants.getEncoderPort(), DriveConstants.kSteerEncoderCAN);
        configCANcoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
        configDriveMotor();

        setDesiredState(new SwerveModuleState(0, getAngle()), false);
    }

    public void periodic() {
        
    }

    public void setDesiredState(SwerveModuleState wantedState, boolean isOpenLoop) {

        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = optimizeStates ? CTREModuleState.optimize(wantedState, getState().angle) : wantedState;
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeed;
            driveMotor.set(percentOutput);
        } else {
            double velocity = ConversionUtils.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.kWheelCircumference,
                                                          DriveConstants.kDriveGearRatio);
            // TODO: This might or might not be the right way to control the motor.
            driveMotor.setControl(new VelocityDutyCycle(velocity));
        }
        if (Constants.DO_LOGGING) {
            double motorSpeed = ConversionUtils.falconToMPS(driveMotor.getVelocity().getValue(), DriveConstants.kWheelCircumference,
                                                            DriveConstants.kDriveGearRatio);
            LogManager.addDouble("Swerve/Modules/DriveSpeed/" + type.name(),
                                 motorSpeed
                                );
            LogManager.addDouble("Swerve/Modules/DriveSpeedError/" + type.name(),
                                 motorSpeed - desiredState.speedMetersPerSecond
                                );
            LogManager.addDouble("Swerve/Modules/DriveVoltage/" + type.name(),
                                 driveMotor.getMotorVoltage().getValue()
                                );
            LogManager.addDouble("Swerve/Modules/DriveCurrent/" + type.name(),
                                 driveMotor.getStatorCurrent().getValue()
                                );
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if desired speed < 1%. Prevents Jittering.
        if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))) {
            stop();
            return;
        }
        angleMotor.setControl(new PositionDutyCycle(ConversionUtils.degreesToFalcon(desiredState.angle.getDegrees(), DriveConstants.kAngleGearRatio)));
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                ConversionUtils.falconToDegrees(angleMotor.getPosition().getValue(), DriveConstants.kAngleGearRatio));
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition().getValue()*360);
    }

    public void resetToAbsolute() {
        // Sensor ticks
        // TODO: Convert sensor ticks in driveconstants to radians
        double absolutePosition = ConversionUtils.degreesToFalcon(getCANcoder().getDegrees() - angleOffset,
                                                                  DriveConstants.kAngleGearRatio);
        angleMotor.setPosition(absolutePosition);                    
    }

    private void configCANcoder() {
        CANcoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(DriveConstants.kModuleConstants.canCoderInvert?SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive));
    }

    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        // TODO: This might or might not be correct
        config.SupplyCurrentLimitEnable = DriveConstants.kAngleEnableCurrentLimit;
        config.SupplyCurrentLimit = DriveConstants.kAngleContinuousCurrentLimit;
        config.SupplyCurrentThreshold = DriveConstants.kAnglePeakCurrentLimit;
        config.SupplyTimeThreshold = DriveConstants.kAnglePeakCurrentDuration;
        angleMotor.getConfigurator().apply(config);
        // TODO: I don't know which config type this is in
        angleMotor.getConfigurator().apply(new SlotConfigs()
            .withKP(DriveConstants.kModuleConstants.angleKP)
            .withKI(DriveConstants.kModuleConstants.angleKI)
            .withKD(DriveConstants.kModuleConstants.angleKD));
        angleMotor.setInverted(DriveConstants.kAngleMotorInvert);
        angleMotor.setNeutralMode(DriveConstants.kAngleNeutralMode);
        angleMotor.setPosition(0);
        m_VelocityVoltage.Slot = 0;
        
        resetToAbsolute();
    }

    public double getSteerVelocity() {
        return ConversionUtils.falconToRPM(angleMotor.getVelocity().getValue(), DriveConstants.kAngleGearRatio) * 2 * Math.PI / 60;
    }
    public double getDriveVelocity() {
        return ConversionUtils.falconToRPM(driveMotor.getVelocity().getValue(), DriveConstants.kAngleGearRatio) * 2 * Math.PI / 60;
    }

    public double getDriveVoltage(){
        return driveMotor.getMotorVoltage().getValue();
    }

    public double getDriveStatorCurrent(){
        return driveMotor.getStatorCurrent().getValue();
    }

    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        // TODO: Again, this might or might not be correct
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.kDriveEnableCurrentLimit;
        config.SupplyCurrentLimit = DriveConstants.kDriveContinuousCurrentLimit;
        config.SupplyCurrentThreshold = DriveConstants.kDrivePeakCurrentLimit;
        config.SupplyTimeThreshold = DriveConstants.kDrivePeakCurrentDuration;
        driveMotor.getConfigurator().apply(config);
        // TODO: Again, I don't know which config type this should be in
        driveMotor.getConfigurator().apply(new SlotConfigs()
            .withKP(DriveConstants.kDriveP)
            .withKI(DriveConstants.kDriveI)
            .withKD(DriveConstants.kDriveD));
        driveMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveConstants.kOpenLoopRamp));
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(DriveConstants.kOpenLoopRamp));
        driveMotor.setInverted(DriveConstants.kDriveMotorInvert);
        driveMotor.setNeutralMode(DriveConstants.kDriveNeutralMode);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(driveMotor.getVelocity().getValue(), DriveConstants.kWheelCircumference,
                                            DriveConstants.kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(driveMotor.getPosition().getValue(), DriveConstants.kWheelCircumference,
                                               DriveConstants.kDriveGearRatio),
                getAngle());
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }


    public double getDriveVelocityError() {
        return getDesiredState().speedMetersPerSecond - getState().speedMetersPerSecond;
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public TalonFX getDriveMotor(){
        return driveMotor;
    }

    public ModuleType getModuleType(){
        return type;
    }

    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
      }
    
      public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
      }
}