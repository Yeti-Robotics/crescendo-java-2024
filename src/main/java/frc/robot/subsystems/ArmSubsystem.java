package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.TalonFXConstants;
import org.opencv.core.Mat;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armKraken;
    private final CANcoder armEncoder;
    private ArmConstants.ArmPositions armPositions = ArmConstants.ArmPositions.STOWED;

    public ArmSubsystem() {
        armKraken = new TalonFX(ArmConstants.ARM_KRAKEN_ID, TalonFXConstants.CANIVORE_NAME);
        armEncoder = new CANcoder(ArmConstants.ARM_CANCODER_ID, TalonFXConstants.CANIVORE_NAME);

        var armConfigurator = armKraken.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();


        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.MotorOutput.Inverted = ArmConstants.ARM_INVERSION;
        talonFXConfiguration.MotorOutput.NeutralMode = ArmConstants.ARM_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = TalonFXConstants.TALON_FUTURE_PROOF;
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
        armPositions = position;
        setMotorsBrake();

        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnits, true, ArmConstants.GRAVITY_FEEDFORWARD * cosineScalar, 0,
                true, false, false);

        armKraken.setControl(motionMagicVoltage);

        SmartDashboard.putNumber("arm set point: ", position.sensorUnits);

    }

    public double getAngle() {
        return armKraken.getRotorPosition().getValue() / CANCoderConstants.COUNTS_PER_DEG * ArmConstants.GEAR_RATIO;
    }

    public ArmConstants.ArmPositions getArmPositions() {
        return armPositions;
    }

    public double getEnc() {
        return armEncoder.getAbsolutePosition().getValue();
    }

    public boolean isMotionFinished() {
        return Math.abs(getAngle() - armPositions.angle) <= ArmConstants.ANGLE_TOLERANCE;
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


