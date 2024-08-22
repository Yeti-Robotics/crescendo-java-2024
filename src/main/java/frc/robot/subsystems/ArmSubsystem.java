package frc.robot.subsystems;


import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armKraken;
    private final CANcoder armEncoder;

    public static class ArmConstants {
        public static ArmConstants.ArmPositions armPositions = ArmConstants.ArmPositions.STOWED;

        public static final int ARM_KRAKEN_ID = 21;
        public static final int ARM_CANCODER_ID = 5;

        public static final InvertedValue ARM_INVERSION = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
            public static final double ARM_POSITION_STATUS_FRAME = 0.05;
        public static final double ARM_VELOCITY_STATUS_FRAME = 0.01;
        public static final double ARM_HANDOFF_POSITION = 0.51;

        public static final double GRAVITY_FEEDFORWARD = 0.05;
        public static final double ARM_P = 0;
        public static final double ARM_I = 0;
        public static final double ARM_D = 0;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ARM_P).withKI(ARM_I).withKD(ARM_D).withGravityType(GravityTypeValue.Arm_Cosine);
        public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

        public static final SoftwareLimitSwitchConfigs ARM_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().
                withForwardSoftLimitEnable
                        (true).
                withForwardSoftLimitThreshold(
                        0.0195 //placeholder
                ).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(
                        65 //placeholder
                );
        public static final double MAGNET_OFFSET = -0.474609;

        public static final double GEAR_RATIO = 1.0 / (50.463 / 12.0);

        public enum ArmPositions {
            STOWED(90);

            public final double angle;
            public final double sensorUnits;

            ArmPositions(double angle) {
                this.angle = angle;
                this.sensorUnits = angle / GEAR_RATIO * Constants.CANCoderConstants.COUNTS_PER_DEG;
            }
        }
    }

    public ArmSubsystem() {
        armKraken = new TalonFX(ArmConstants.ARM_KRAKEN_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        armEncoder = new CANcoder(ArmConstants.ARM_CANCODER_ID, Constants.TalonFXConstants.CANIVORE_NAME);

        var armConfigurator = armKraken.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();


        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.MotorOutput.Inverted = ArmConstants.ARM_INVERSION;
        talonFXConfiguration.MotorOutput.NeutralMode = ArmConstants.ARM_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = Constants.TalonFXConstants.TALON_FUTURE_PROOF;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 50; //placeholder
        talonFXConfiguration.Feedback.RotorToSensorRatio = 12.8;
        talonFXConfiguration.CurrentLimits = ArmConstants.ARM_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = ArmConstants.ARM_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = ArmConstants.SLOT_0_CONFIGS;

        armKraken.getRotorVelocity().waitForUpdate(ArmConstants.ARM_VELOCITY_STATUS_FRAME);
        armKraken.getRotorPosition().waitForUpdate(ArmConstants.ARM_POSITION_STATUS_FRAME);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = ArmConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armEncoderConfigurator.apply(cancoderConfiguration);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder: ", armEncoder.getAbsolutePosition().getValue());
    }


    public void setPosition(ArmConstants.ArmPositions position) {
        ArmConstants.armPositions = position;
        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnits, true, ArmConstants.GRAVITY_FEEDFORWARD * cosineScalar, 0,
                true, false, false);

        armKraken.setControl(motionMagicVoltage);

        SmartDashboard.putNumber("arm set point: ", position.sensorUnits);

    }

    private double getAngle() {
        return armKraken.getRotorPosition().getValue() / Constants.CANCoderConstants.COUNTS_PER_DEG * ArmConstants.GEAR_RATIO;
    }

    public double getEnc() {
        return armEncoder.getAbsolutePosition().getValue();
    }

    private void moveUp(double speed) {
        armKraken.set(Math.abs(speed));
    }

    public Command moveUpAndStop(double speed) {
        return startEnd(() -> moveUp(speed), this::stop);
    }

    public Command moveDownAndStop(double speed){
        return startEnd(() -> moveDown(speed), this::stop);
    }

    private void moveDown(double speed) {
        armKraken.set(-Math.abs(speed));
    }

    private void stop() {
        armKraken.stopMotor();
    }

}


