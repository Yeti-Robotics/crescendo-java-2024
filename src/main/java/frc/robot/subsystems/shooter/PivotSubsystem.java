package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
    public final TalonFX pivotMotor;
    public final CANcoder pivotEncoder;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;

    private final StatusSignal<Double> pivotPositionStatusSignal;

    public static class PivotConstants {
        public static final int PIVOT_LIMIT_SWITCH_FORWARD = 7;
        public static final int PIVOT_LIMIT_SWITCH_REVERSE = 6;
        public static final int PIVOT_ONE_MOTOR_ID = 29; // placeholder
        public static final int PIVOT_ONE_CANCODER_ID = 16; // placeholder
        public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final double PIVOT_P = 50; // 1 //350.0 /11.7
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 2; // 0 //45.0
        public static final double PIVOT_V = 0.12000000149011612; // 65
        public static final double PIVOT_A = 0.009999999776482582; // 0.7
        public static final double PIVOT_G = 0.02; // 0.35
        public static final double PROFILE_V = 0.000000001;
        public static final double PROFILE_A = 0.000000001;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(PIVOT_P).withKI(PIVOT_I)
                .withKD(PIVOT_D).withKA(PIVOT_A).withKV(PIVOT_V).withKG(PIVOT_G)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMIT = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65)
                .withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

        public static final SoftwareLimitSwitchConfigs PIVOT_SOFT_LIMIT = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(
                        .53)
                .withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(
                        .31);

        public static final double MAGNET_OFFSET = 0.433838; // placeholder
        public static final double ANGLE_THRESHOLD = 0.01;

        public enum PivotPosition {
            BUMP_FIRE(0.53),
            HANDOFF(0.5);

            // placeholder
            private double position;

            PivotPosition(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public PivotSubsystem() {
        reverseLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_REVERSE);
        forwardLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_FORWARD);
        pivotMotor = new TalonFX(PivotConstants.PIVOT_ONE_MOTOR_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        pivotEncoder = new CANcoder(PivotConstants.PIVOT_ONE_CANCODER_ID, Constants.TalonFXConstants.CANIVORE_NAME);

        pivotMotor.setInverted(true);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotPositionStatusSignal = pivotMotor.getPosition();

        new Trigger(this::getForwardLimitSwitch).whileTrue(moveDown(0.05));
        new Trigger(this::getReverseLimitSwitch).whileTrue(moveUp(0.05));

        var pivotMotorConfigurator = pivotMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = PivotConstants.PIVOT_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 83.79;
        talonFXConfiguration.CurrentLimits = PivotConstants.PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = PivotConstants.PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = PivotConstants.SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 2;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = PivotConstants.PROFILE_A;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = PivotConstants.PROFILE_V;

        pivotMotorConfigurator.apply(talonFXConfiguration);

        var pivotEncoderConfigurator = pivotEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoderConfigurator.apply(cancoderConfiguration);
    }

    @Override
    public void periodic() {
        pivotPositionStatusSignal.refresh();

        SmartDashboard.putNumber("Pivot pos:", getPosition());
    }

    public double getPosition() {
        return pivotPositionStatusSignal.getValueAsDouble();
    }

    private boolean getForwardLimitSwitch() {
        return !forwardLimitSwitch.get();
    }

    private boolean getReverseLimitSwitch() {
        return !reverseLimitSwitch.get();
    }

    private void setPosition(double pivotPosition) {
        MotionMagicVoltage motionMagicControlRequest = new MotionMagicVoltage(
                pivotPosition, true, 0, 0, false, false, false)
                .withLimitForwardMotion(getForwardLimitSwitch())
                .withLimitReverseMotion(getReverseLimitSwitch())
                .withSlot(0)
                .withUpdateFreqHz(200);

        pivotMotor.setControl(motionMagicControlRequest);
    }

    private void movePivotMotorUp(double speed) {
        if (!getForwardLimitSwitch()) {
            pivotMotor.set(Math.abs(speed));
        }
    }

    private void movePivotMotorDown(double speed) {
        if (!getReverseLimitSwitch()) {
            pivotMotor.set(-Math.abs(speed));
        }
    }

    private void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    public Command adjustPivotPositionTo(PivotConstants.PivotPosition pivotPosition) {
        return adjustPivotPositionTo(pivotPosition.getPosition());
    }

    public Command adjustPivotPositionTo(Supplier<Double> positionSupplier) {
        return runEnd(() -> setPosition(positionSupplier.get()), this::stopPivotMotor);
    }

    public Command adjustPivotPositionTo(double position) {
        return runOnce(() -> setPosition(position));
    }

    public Command moveUp(double speed) {
        return startEnd(() -> movePivotMotorUp(speed), this::stopPivotMotor);
    }

    public Command moveDown(double speed) {
        return startEnd(() -> movePivotMotorDown(speed), this::stopPivotMotor);
    }

    public Command moveDownWithBrake(double moveSpeed, double brakeSpeed) {
        return startEnd(() -> movePivotMotorDown(moveSpeed), () -> movePivotMotorUp(brakeSpeed));
    }

    public Command moveUpWithBrake(double moveSpeed, double brakeSpeed) {
        return startEnd(() -> movePivotMotorUp(moveSpeed), () -> movePivotMotorDown(brakeSpeed));
    }
}
