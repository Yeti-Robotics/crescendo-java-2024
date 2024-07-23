package frc.robot.subsystems;


import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConfiguratorConstants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armKraken;
    private final CANcoder armEncoder;
    public static final double GEAR_RATIO = 1.0 / (50.463 / 12.0);
    public enum ArmPositions {
        STOWED(90);
        public final double angle;
        public final double sensorUnits;
        ArmPositions(double angle) {
            this.angle = angle;
            this.sensorUnits = angle / GEAR_RATIO * ConfiguratorConstants.COUNTS_PER_DEG;
        }
    }
    private ArmPositions armPositions = ArmPositions.STOWED;
    private static final double ARM_POSITION_STATUS_FRAME = 0.05;
    private static final double ARM_VELOCITY_STATUS_FRAME = 0.01;
    public static final double ARM_HANDOFF_POSITION = 0.03;
    private static final double GRAVITY_FEEDFORWARD = 0.05;
    private static final double ANGLE_TOLERANCE = 5;
    public static final double ARM_P = 0;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ARM_P).withKI(ARM_I).withKD(ARM_D).withGravityType(GravityTypeValue.Arm_Cosine);
    public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
    public static final double MAGNET_OFFSET = -1; //placeholder
    public static final InvertedValue ARM_INVERSION = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final SoftwareLimitSwitchConfigs ARM_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().
            withForwardSoftLimitEnable
                    (true).
            withForwardSoftLimitThreshold(
                    0.0195 //placeholder
            ).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(
                    65 //placeholder
            );

    public ArmSubsystem() {
        armKraken = new TalonFX(ConfiguratorConstants.ARM_KRAKEN_ID, ConfiguratorConstants.CANIVORE_NAME);
        armEncoder = new CANcoder(ConfiguratorConstants.ARM_CANCODER_ID, ConfiguratorConstants.CANIVORE_NAME);
        var armConfigurator = armKraken.getConfigurator();
        var armTalonFXConfiguration = new TalonFXConfiguration();
        armTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        armTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armTalonFXConfiguration.MotorOutput.Inverted = ARM_INVERSION;
        armTalonFXConfiguration.MotorOutput.NeutralMode = ARM_NEUTRAL_MODE;
        armTalonFXConfiguration.FutureProofConfigs = ConfiguratorConstants.TALON_FUTURE_PROOF;
        armTalonFXConfiguration.Feedback.SensorToMechanismRatio = 50; //placeholder
        armTalonFXConfiguration.Feedback.RotorToSensorRatio = 12.8;
        armTalonFXConfiguration.CurrentLimits = ARM_CURRENT_LIMIT;
        armTalonFXConfiguration.SoftwareLimitSwitch = ARM_SOFT_LIMIT;
        armTalonFXConfiguration.Slot0 = SLOT_0_CONFIGS;

        armKraken.getRotorVelocity().waitForUpdate(ARM_VELOCITY_STATUS_FRAME);
        armKraken.getRotorPosition().waitForUpdate(ARM_POSITION_STATUS_FRAME);

        armConfigurator.apply(armTalonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoderConfigurator.apply(cancoderConfiguration);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder: ", armEncoder.getAbsolutePosition().getValue());
    }


    public void setPosition(ArmPositions position) {
        armPositions = position;
        setMotorsBrake();

        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnits, true, GRAVITY_FEEDFORWARD * cosineScalar, 0,
                true, false, false);

        armKraken.setControl(motionMagicVoltage);

        SmartDashboard.putNumber("arm set point: ", position.sensorUnits);

    }

    public double getAngle() {
        return armKraken.getRotorPosition().getValue() / ConfiguratorConstants.COUNTS_PER_DEG * GEAR_RATIO;
    }

    public ArmPositions getArmPositions() {
        return armPositions;
    }

    public double getEnc() {
        return armEncoder.getAbsolutePosition().getValue();
    }

    public boolean isMotionFinished() {
        return Math.abs(getAngle() - armPositions.angle) <= ANGLE_TOLERANCE;
    }

    public void moveUp(double speed) {
        setMotorsBrake();
        armKraken.set(Math.abs(speed));
    }

    public void moveDown(double speed) {
        armKraken.set(-Math.abs(speed));
    }

    public void setMotorsCoast() {
        armKraken.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setMotorsBrake() {
        armKraken.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getSuppliedCurrent() {
        return armKraken.getSupplyCurrent().getValue();
    }

    public void stop() {
        armKraken.stopMotor();
    }

    public boolean atAngle() {
        return (armEncoder.getAbsolutePosition().getValue() < 0.18261);
    }


}


