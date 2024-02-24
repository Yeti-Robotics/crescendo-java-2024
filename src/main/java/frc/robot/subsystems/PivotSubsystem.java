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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.TalonFXConstants;
import org.opencv.core.Mat;

import static frc.robot.constants.FieldConstants.*;

public class PivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor1;
    private final CANcoder pivotEncoder1;
    private final VisionSubsystem visionSubsystem;
    private double relativePoseY;
    private double relativePoseX;
    private double robotPoseY;
    private double robotPoseX;
    private double hypoGroundLength;
    public double vertAngle;
    private PivotConstants.PivotPositions pivotPositions = PivotConstants.PivotPositions.BUMPFIRE;
    public PivotSubsystem() {
        visionSubsystem = new VisionSubsystem();
        pivotMotor1 = new TalonFX(PivotConstants.PIVOT_ONE_MOTOR_ID, TalonFXConstants.CANIVORE_NAME);
        pivotEncoder1 = new CANcoder(PivotConstants.PIVOT_ONE_CANCODER_ID, TalonFXConstants.CANIVORE_NAME);

        var pivotMotor1Configurator = pivotMotor1.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder1.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = PivotConstants.PIVOT_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 12.8;
        talonFXConfiguration.CurrentLimits = PivotConstants.PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = PivotConstants.PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = PivotConstants.SLOT_0_CONFIGS;

        pivotMotor1.getRotorVelocity().waitForUpdate(PivotConstants.PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor1.getRotorPosition().waitForUpdate(PivotConstants.PIVOT_POSITION_STATUS_FRAME);


        pivotMotor1Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder1.getConfigurator();

        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder1Configurator.apply(cancoderConfiguration);
    }
    @Override
    public void periodic() {
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = speakerPose - visionSubsystem.getPose2d().getX();
        robotPoseX = visionSubsystem.getPose2d().getX();
        robotPoseY = visionSubsystem.getPose2d().getY();
        hypoGroundLength = Math.sqrt((relativePoseX*relativePoseX)+(relativePoseY*relativePoseY));
        vertAngle = Math.atan2(Units.inchesToMeters(speakerHeightRelativeToBot), hypoGroundLength);
    }

    public void setPivotPosition(double angle) {

        double angleUnits = angle/PivotConstants.GEAR_RATIO/360;
        setMotorsBrake();

        double radians = Math.toRadians(getAngle());
        double cosineScalar = Math.cos(radians);



        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                angleUnits, true, PivotConstants.GRAVITY_FEEDFORWARD * cosineScalar, 0,
                false, false, false);

        System.out.println(angleUnits);
        System.out.println(motionMagicVoltage.Position);

        pivotMotor1.setControl(motionMagicVoltage);
    }

    public double getAngle() {
        return pivotMotor1.getRotorPosition().getValue() / CANCoderConstants.COUNTS_PER_DEG * PivotConstants.GEAR_RATIO;
    }

    public double getEncAngle() {
        return pivotEncoder1.getAbsolutePosition().getValue();
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
    }
    public void moveDown(double speed) {
        setMotorsBrake();
        pivotMotor1.set(-Math.abs(speed));
    }

    public void setMotorsCoast() {
        pivotMotor1.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setMotorsBrake() {
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getSuppliedCurrent() {
        return pivotMotor1.getSupplyCurrent().getValue();
    }

    public void stop() {
        pivotMotor1.stopMotor();
    }

}

