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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.TalonFXConstants;
import org.opencv.core.Mat;

public class PivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;
    private final CANcoder pivotEncoder1;
    private final CANcoder pivotEncoder2;
    private PivotConstants.PivotPositions pivotPositions = PivotConstants.PivotPositions.BUMPFIRE;
    public PivotSubsystem() {
        pivotMotor1 = new TalonFX(PivotConstants.PIVOT_ONE_MOTOR_ID);
        pivotMotor2 = new TalonFX(PivotConstants.PIVOT_TWO_MOTOR_ID);
        pivotEncoder1 = new CANcoder(PivotConstants.PIVOT_ONE_CANCODER_ID);
        pivotEncoder2 = new CANcoder(PivotConstants.PIVOT_TWO_CANCODER_ID);

        var pivotMotor1Configurator = pivotMotor1.getConfigurator();
        var pivotMotor2Configurator = pivotMotor2.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();


        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder1.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder2.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = PivotConstants.PIVOT_NEUTRAL_MODE;
        // talonFXConfiguration.FutureProofConfigs = TalonFXConstants.TALON_FUTURE_PROOF;
        // talonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        // talonFXConfiguration.Feedback.RotorToSensorRatio = 12.8;
        talonFXConfiguration.CurrentLimits = PivotConstants.PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = PivotConstants.PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = PivotConstants.SLOT_0_CONFIGS;

        pivotMotor1.getRotorVelocity().waitForUpdate(PivotConstants.PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor2.getRotorVelocity().waitForUpdate(PivotConstants.PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor1.getRotorPosition().waitForUpdate(PivotConstants.PIVOT_POSITION_STATUS_FRAME);
        pivotMotor2.getRotorPosition().waitForUpdate(PivotConstants.PIVOT_POSITION_STATUS_FRAME);


        pivotMotor1Configurator.apply(talonFXConfiguration);
        pivotMotor2Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder1.getConfigurator();
        var pivotEncoder2Configurator = pivotEncoder2.getConfigurator();

        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder1Configurator.apply(cancoderConfiguration);
        pivotEncoder2Configurator.apply(cancoderConfiguration);
    }
    @Override
    public void periodic() {    }

    public void setPosition(PivotConstants.PivotPositions position) {
        pivotPositions = position;
        setMotorsBrake();

        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnits, true, PivotConstants.GRAVITY_FEEDFORWARD * cosineScalar, 0,
                true, false, false);

        pivotMotor1.setControl(motionMagicVoltage);
        pivotMotor2.setControl(motionMagicVoltage);
    }

    public double getAngle() {
        return pivotMotor1.getRotorPosition().getValue() / CANCoderConstants.COUNTS_PER_DEG * PivotConstants.GEAR_RATIO;
    }

    public PivotConstants.PivotPositions getArmPositions() {
        return pivotPositions;
    }

    public boolean isMotionFinished() {
        return Math.abs(getAngle() - pivotPositions.angle) <= PivotConstants.ANGLE_TOLERANCE;
    }

    public void moveUp(double speed) {
        setMotorsBrake();
        pivotMotor1.set(Math.abs(speed));
        pivotMotor2.set(Math.abs(speed));
    }
    public void moveDown(double speed) {
        setMotorsBrake();
        pivotMotor1.set(-Math.abs(speed));
        pivotMotor2.set(-Math.abs(speed));
    }

    public void setMotorsCoast() {
        pivotMotor1.setNeutralMode(NeutralModeValue.Coast);
        pivotMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setMotorsBrake() {
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getSuppliedCurrent() {
        return pivotMotor1.getSupplyCurrent().getValue();
    }

    public void stop() {
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }

}
