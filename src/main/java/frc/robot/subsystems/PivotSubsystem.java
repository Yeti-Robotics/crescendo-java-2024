package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants.PivotPosition;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.RobotDataPublisher.RobotDataSubscription;
import frc.robot.util.ShooterStateData;

public class PivotSubsystem extends SubsystemBase {
    public final TalonFX pivotMotor;
    public final CANcoder pivotEncoder;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;

    private PivotPosition pivotSetPosition = Constants.PivotConstants.PivotPosition.CUSTOM(0.45);
    private final StatusSignal<Double> pivotPositionStatusSignal;

    public PivotSubsystem() {
        reverseLimitSwitch = new DigitalInput(Constants.PivotConstants.PIVOT_LIMIT_SWITCH_REVERSE);
        forwardLimitSwitch = new DigitalInput(Constants.PivotConstants.PIVOT_LIMIT_SWITCH_FORWARD);
        pivotMotor = new TalonFX(Constants.PivotConstants.PIVOT_ONE_MOTOR_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        pivotEncoder = new CANcoder(Constants.PivotConstants.PIVOT_ONE_CANCODER_ID, Constants.TalonFXConstants.CANIVORE_NAME);

        pivotMotor.setInverted(true);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotPositionStatusSignal = pivotMotor.getPosition();

        new Trigger(this::getForwardLimitSwitch).whileTrue(moveDown(0.05));
        new Trigger(this::getReverseLimitSwitch).whileTrue(moveUp(0.05));

        var pivotMotor1Configurator = pivotMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = Constants.PivotConstants.PIVOT_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 83.79;
        talonFXConfiguration.CurrentLimits = Constants.PivotConstants.PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = Constants.PivotConstants.PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = Constants.PivotConstants.SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 2;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = Constants.PivotConstants.PROFILE_A;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = Constants.PivotConstants.PROFILE_V;

        pivotMotor1Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = Constants.PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoder1Configurator.apply(cancoderConfiguration);
    }

    @Override
    public void periodic() {
        pivotPositionStatusSignal.refresh();

        Sendable pivotPositionSendable = sendableBuilder -> {
            sendableBuilder.addStringProperty("Type", pivotSetPosition::toString, null);
            sendableBuilder.addDoubleProperty("Position", pivotSetPosition::getPosition, null);
        };

        SmartDashboard.putNumber("Pivot pos:", getPivotPosition());

    }

    public double getPivotPosition() {
        if (!pivotPositionStatusSignal.hasUpdated()) {
            pivotPositionStatusSignal.waitForUpdate(Constants.PivotConstants.PIVOT_POSITION_STATUS_FRAME);
        }

        return pivotPositionStatusSignal.getValue();
    }

    private void setPivotPosition(PivotPosition pivotPosition) {
        pivotSetPosition = pivotPosition;

        MotionMagicVoltage motionMagicControlRequest = new MotionMagicVoltage(
                pivotPosition.getPosition(), true, 0, 0,
                false, false, false)
                .withLimitForwardMotion(getForwardLimitSwitch())
                .withLimitReverseMotion(getReverseLimitSwitch())
                .withSlot(0).withUpdateFreqHz(200);

        pivotMotor.setControl(motionMagicControlRequest);
    }

    public Command movePivotPositionTo(PivotPosition pivotPosition) {
        return runOnce(() -> setPivotPosition(pivotPosition));
    }

    public Command adjustPivotPositionTo(double angle) {
        return runOnce(() -> setPivotPosition(PivotPosition.CUSTOM(angle)));
    }

    public Command updatePivotPositionWith(RobotDataPublisher<ShooterStateData> shooterDataPublisher) {
        RobotDataSubscription<ShooterStateData> shooterStateSubscription = shooterDataPublisher.subscribeWith(data -> setPivotPosition(PivotPosition.CUSTOM(data.angle)));

        return startEnd(shooterStateSubscription::start, shooterStateSubscription::cancel);
    }

    public double getEncoderAngle() {
        return pivotEncoder.getAbsolutePosition().getValue();
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

    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    public boolean getForwardLimitSwitch() {
        return !forwardLimitSwitch.get();
    }

    public boolean getReverseLimitSwitch() {
        return !reverseLimitSwitch.get();
    }
}

