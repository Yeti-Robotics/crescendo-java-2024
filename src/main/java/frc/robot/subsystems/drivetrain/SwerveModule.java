package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.constants.DriveConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final CANcoder absoluteEncoder;
    private final PIDController drivePIDController = new PIDController(
            DriveConstants.DRIVE_MOTOR_P,
            DriveConstants.DRIVE_MOTOR_I,
            DriveConstants.DRIVE_MOTOR_D);
    private final ProfiledPIDController azimuthPIDController = new ProfiledPIDController(
            DriveConstants.AZIMUTH_MOTOR_P,
            DriveConstants.AZIMUTH_MOTOR_I,
            DriveConstants.AZIMUTH_MOTOR_D,
            new TrapezoidProfile.Constraints(3 * Math.PI, 6 * Math.PI));
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            DriveConstants.DRIVE_MOTOR_KS, DriveConstants.DRIVE_MOTOR_KV, DriveConstants.DRIVE_MOTOR_KA
    );

    private final SimpleMotorFeedforward azimuthFeedForward = new SimpleMotorFeedforward(
            DriveConstants.AZIMUTH_MOTOR_KS, DriveConstants.AZIMUTH_MOTOR_KV, DriveConstants.AZIMUTH_MOTOR_KA
    );
    private final SwerveModuleState state = new SwerveModuleState();
    private final SwerveModulePosition position = new SwerveModulePosition();


    public SwerveModule(
            int driveMotorID,
            boolean driveInverted,
            int steerMotorID,
            int absoluteEncoderID,
            SensorDirectionValue absoluteEncoderReversed,
            double absoluteEncoderOffsetDeg) {

        absoluteEncoder = new CANcoder(absoluteEncoderID);
        var absoluteEncoderConfig = new CANcoderConfiguration();
        var configurator = absoluteEncoder.getConfigurator();
        absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absoluteEncoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDeg;
        absoluteEncoderConfig.MagnetSensor.SensorDirection = absoluteEncoderReversed;
        absoluteEncoderConfig.FutureProofConfigs = true;
        configurator.apply(absoluteEncoderConfig);

        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);

        var driveConfig = new TalonFXConfiguration();
        var driveConfigurator = driveMotor.getConfigurator();
        var steerConfig = new TalonFXConfiguration();
        var steerConfigurator = steerMotor.getConfigurator();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.setInverted(driveInverted);
        steerMotor.setInverted(true);

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        driveConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
        driveConfig.CurrentLimits.StatorCurrentLimit = 60;
        steerConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
        steerConfig.CurrentLimits.StatorCurrentLimit = 60;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 65;
        steerConfig.CurrentLimits.SupplyCurrentLimit = 65;

        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveMotor.getRotorVelocity().waitForUpdate(250);
        driveMotor.getRotorPosition().waitForUpdate(20);

        steerMotor.getRotorVelocity().waitForUpdate(250);
        steerMotor.getRotorPosition().waitForUpdate(250);

        driveConfigurator.apply(driveConfig);
        steerConfigurator.apply(steerConfig);

        azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }


    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValue() / 2048 *
                DriveConstants.SWERVE_X_REDUCTION *
                DriveConstants.WHEEL_DIAMETER * Math.PI;
    }

    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() * 10 / 2048 *
                (DriveConstants.WHEEL_DIAMETER * Math.PI) * DriveConstants.SWERVE_X_REDUCTION;
    }
    public double getAzimuthPosition() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getState() {
        state.speedMetersPerSecond = getDriveVelocity();
        state.angle = new Rotation2d(getAzimuthPosition());
        return state;
    }
    public SwerveModulePosition getPosition() {
        position.distanceMeters = getDrivePosition();
        position.angle = new Rotation2d(getAzimuthPosition());
        return position;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double driveVelocity = getDriveVelocity();
        double azimuthPosition = getAzimuthPosition();

        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(azimuthPosition));
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
                && Math.abs(desiredState.angle.getRadians() - azimuthPosition) < 0.05) {
            stop();
            return;
        }

        final double driveOutput =
                drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
                        + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        final double azimuthOutput =
                azimuthPIDController.calculate(azimuthPosition, desiredState.angle.getRadians())
                        + azimuthFeedForward.calculate(azimuthPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput);
        steerMotor.setVoltage(azimuthOutput);
    }

    public void stop() {
        driveMotor.setVoltage(0.0);
        steerMotor.set(0.0);
    }
}
