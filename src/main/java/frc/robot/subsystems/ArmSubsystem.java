package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armKraken;
    private final CANcoder armEncoder;
    private Constants.ArmConstants.ArmPositions armPositions = Constants.ArmConstants.ArmPositions.STOWED;

    public ArmSubsystem() {
        armKraken = new TalonFX(Constants.ArmConstants.ARM_KRAKEN_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        armEncoder = new CANcoder(Constants.ArmConstants.ARM_CANCODER_ID, Constants.TalonFXConstants.CANIVORE_NAME);

        var armConfigurator = armKraken.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();


        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.MotorOutput.Inverted = Constants.ArmConstants.ARM_INVERSION;
        talonFXConfiguration.MotorOutput.NeutralMode = Constants.ArmConstants.ARM_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = Constants.TalonFXConstants.TALON_FUTURE_PROOF;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 50; //placeholder
        talonFXConfiguration.Feedback.RotorToSensorRatio = 12.8;
        talonFXConfiguration.CurrentLimits = Constants.ArmConstants.ARM_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = Constants.ArmConstants.ARM_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = Constants.ArmConstants.SLOT_0_CONFIGS;

        armKraken.getRotorVelocity().waitForUpdate(Constants.ArmConstants.ARM_VELOCITY_STATUS_FRAME);
        armKraken.getRotorPosition().waitForUpdate(Constants.ArmConstants.ARM_POSITION_STATUS_FRAME);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = Constants.ArmConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoderConfigurator.apply(cancoderConfiguration);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder: ", armEncoder.getAbsolutePosition().getValue());
    }


    public void setPosition(Constants.ArmConstants.ArmPositions position) {
        armPositions = position;
        setMotorsBrake();

        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnits, true, Constants.ArmConstants.GRAVITY_FEEDFORWARD * cosineScalar, 0,
                true, false, false);

        armKraken.setControl(motionMagicVoltage);

        SmartDashboard.putNumber("arm set point: ", position.sensorUnits);

    }

    public double getAngle() {
        return armKraken.getRotorPosition().getValue() / Constants.CANCoderConstants.COUNTS_PER_DEG * Constants.ArmConstants.GEAR_RATIO;
    }

    public Constants.ArmConstants.ArmPositions getArmPositions() {
        return armPositions;
    }

    public double getEnc() {
        return armEncoder.getAbsolutePosition().getValue();
    }

    public boolean isMotionFinished() {
        return Math.abs(getAngle() - armPositions.angle) <= Constants.ArmConstants.ANGLE_TOLERANCE;
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


