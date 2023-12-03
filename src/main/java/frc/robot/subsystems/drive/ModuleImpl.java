package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.GlobalConst;
import frc.robot.constants.swerve.DriveConst;
import frc.robot.constants.swerve.ModuleConst;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.ConversionUtils;
import frc.robot.util.LogManager;
import lib.CTREModuleState;

public class ModuleImpl extends Module {
    private final ModuleType type;
    
    // Motor ticks
    private final double angleOffset;

    private final WPI_TalonFX angleMotor;
    private final WPI_TalonFX driveMotor;
    private final WPI_CANCoder CANcoder;
    private SwerveModuleState desiredState;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConst.DRIVE_KS, DriveConst.DRIVE_KV, DriveConst.DRIVE_KA);

    private boolean optimizeStates = true;

    public ModuleImpl(ModuleConst ModuleConst) {
        super(ModuleConst);

        type = ModuleConst.getType();

//        angleOffset = new Rotation2d(constants.getSteerOffset());
        angleOffset = ModuleConst.getSteerOffset();

        /* Angle Encoder Config */
        CANcoder = new WPI_CANCoder(ModuleConst.getEncoderPort(), DriveConst.kSteerEncoderCAN);
        configCANcoder();

        /* Angle Motor Config */
        angleMotor = new WPI_TalonFX(ModuleConst.getSteerPort(), DriveConst.kSteerEncoderCAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new WPI_TalonFX(ModuleConst.getDrivePort(), DriveConst.kDriveMotorCAN);
        configDriveMotor();

        setDesiredState(new SwerveModuleState(0, getAngle()), false);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        this.desiredState = optimizeStates ? CTREModuleState.optimize(desiredState, getState().angle) : desiredState;
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConst.kMaxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = ConversionUtils.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConst.kWheelCircumference,
                                                          DriveConst.kDriveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                           feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        if (GlobalConst.DO_LOGGING) {
            double motorSpeed = ConversionUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConst.kWheelCircumference,
                                                            DriveConst.kDriveGearRatio);
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
        if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConst.kMaxSpeed * 0.01))) {
            stop();
            return;
        }
        angleMotor.set(ControlMode.Position, ConversionUtils.degreesToFalcon(desiredState.angle.getDegrees(), DriveConst.kAngleGearRatio));
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                ConversionUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConst.kAngleGearRatio));
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition());
    }

    @Override
    public void resetToAbsolute() {
        // Sensor ticks
        // TODO: Convert sensor ticks in DriveConst to radians
        double absolutePosition = ConversionUtils.degreesToFalcon(getCANcoder().getDegrees() - angleOffset,
                                                                  DriveConst.kAngleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);                                                         
    }

    private void configCANcoder() {
        CANcoder.configFactoryDefault();
        CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        CANcoder.configSensorDirection(DriveConst.kModuleConstants.canCoderInvert);
        CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        CANcoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                DriveConst.kAngleEnableCurrentLimit,
                DriveConst.kAngleContinuousCurrentLimit,
                DriveConst.kAnglePeakCurrentLimit,
                DriveConst.kAnglePeakCurrentDuration
        ));
        angleMotor.config_kP(0, DriveConst.kModuleConstants.angleKP);
        angleMotor.config_kI(0, DriveConst.kModuleConstants.angleKI);
        angleMotor.config_kD(0, DriveConst.kModuleConstants.angleKD);
        angleMotor.config_kF(0, DriveConst.kModuleConstants.angleKF);
        angleMotor.setInverted(DriveConst.kAngleMotorInvert);
        angleMotor.setNeutralMode(DriveConst.kAngleNeutralMode);
        angleMotor.configVoltageCompSaturation(GlobalConst.ROBOT_VOLTAGE);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setSelectedSensorPosition(0);
        resetToAbsolute();
    }

    public double getSteerVelocity() {
        return ConversionUtils.falconToRPM(angleMotor.getSelectedSensorVelocity(), DriveConst.kAngleGearRatio) * 2 * Math.PI / 60;
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                DriveConst.kDriveEnableCurrentLimit,
                DriveConst.kDriveContinuousCurrentLimit,
                DriveConst.kDrivePeakCurrentLimit,
                DriveConst.kDrivePeakCurrentDuration
        ));
        driveMotor.config_kP(0, DriveConst.kDriveP);
        driveMotor.config_kI(0, DriveConst.kDriveI);
        driveMotor.config_kD(0, DriveConst.kDriveD);
        driveMotor.config_kF(0, DriveConst.kDriveF);
        driveMotor.configOpenloopRamp(DriveConst.kOpenLoopRamp);
        driveMotor.configClosedloopRamp(DriveConst.kClosedLoopRamp);
        driveMotor.setInverted(DriveConst.kDriveMotorInvert);
        driveMotor.setNeutralMode(DriveConst.kDriveNeutralMode);
        driveMotor.configVoltageCompSaturation(GlobalConst.ROBOT_VOLTAGE);
        driveMotor.enableVoltageCompensation(true);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConst.kWheelCircumference,
                                            DriveConst.kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(driveMotor.getSelectedSensorPosition(), DriveConst.kWheelCircumference,
                                               DriveConst.kDriveGearRatio),
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

    public WPI_TalonFX getDriveMotor(){
        return driveMotor;
    }
}