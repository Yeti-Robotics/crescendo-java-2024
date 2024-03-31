package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.TalonFXConstants;
import org.opencv.core.Mat;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.FieldConstants.*;

public class PivotSubsystem extends SubsystemBase {

    public final TalonFX pivotMotor1;
    public final CANcoder pivotEncoder1;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;
    private final VisionSubsystem visionSubsystem;
    private double relativePoseY;
    private double relativePoseX;
    private double robotPoseY;
    private double robotPoseX;
    private double hypoGroundLength;
    public double vertAngle;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private PivotConstants.PivotPositions pivotPositions = PivotConstants.PivotPositions.BUMPFIRE;

    private static final Measure<Velocity<Voltage>> sysIdRampRate =
            edu.wpi.first.units.Units.Volts.of(1).per(Seconds.of(1));
    private static final Measure<Voltage> sysIdStepAmps = edu.wpi.first.units.Units.Volts.of(7);
    // SysID Setup
    private final SysIdRoutine sysIdRoutine;

    public PivotSubsystem() {
        visionSubsystem = new VisionSubsystem();
        reverseLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_REVERSE);
        forwardLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_FORWARD);
        pivotMotor1 = new TalonFX(PivotConstants.PIVOT_ONE_MOTOR_ID, TalonFXConstants.CANIVORE_NAME);
        pivotEncoder1 = new CANcoder(PivotConstants.PIVOT_ONE_CANCODER_ID, TalonFXConstants.CANIVORE_NAME);
        pivotMotor1.setInverted(true);

        var pivotMotor1Configurator = pivotMotor1.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder1.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = PivotConstants.PIVOT_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 83.79;
        talonFXConfiguration.CurrentLimits = PivotConstants.PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = PivotConstants.PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = PivotConstants.SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = PivotConstants.PROFILE_A;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = PivotConstants.PROFILE_V;

        pivotMotor1.getRotorVelocity().waitForUpdate(PivotConstants.PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor1.getRotorPosition().waitForUpdate(PivotConstants.PIVOT_POSITION_STATUS_FRAME);


        pivotMotor1Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder1.getConfigurator();

        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder1Configurator.apply(cancoderConfiguration);

        sysIdRoutine =
                new SysIdRoutine(
                        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                        new SysIdRoutine.Config(
                                sysIdRampRate,
                                sysIdStepAmps,
                                null,
                                state -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                // Tell SysId how to plumb the driving voltage to the motor(s).
                                (Measure<Voltage> volts) -> {
                                    pivotMotor1.setControl(
                                            voltageRequest.withOutput(volts.in(edu.wpi.first.units.Units.Volts)));
                                },
                                // Tell SysId how to record a frame of data for each motor on the mechanism being
                                // characterized.
                                null, // Using the CTRE SignalLogger API instead
                                // Tell SysId to make generated commands require this subsystem, suffix test state in
                                // WPILog with this subsystem's name ("shooter")
                                this));
    }
    @Override
    public void periodic() {
//        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
//        relativePoseX = speakerPose - visionSubsystem.getPose2d().getX();
//        robotPoseX = visionSubsystem.getPose2d().getX();
//        robotPoseY = visionSubsystem.getPose2d().getY();
//        hypoGroundLength = Math.sqrt((relativePoseX*relativePoseX)+(relativePoseY*relativePoseY));
//        vertAngle = Math.atan2(Units.inchesToMeters(speakerHeightRelativeToBot), hypoGroundLength);

//        SmartDashboard.putData("Pivot kraken", pivotMotor1);
        SmartDashboard.putData("Pivot encoder", pivotEncoder1);


    }

    public void setPivotPosition(double position) {
        MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(
                position, true, 0, 0,
                false, false, false);
        // todo: overridebreakdurneutral = false

        System.out.print("Pivot position: ");
        System.out.println(position);
        System.out.println(motionMagic.Position);
        pivotMotor1.setControl(motionMagic.withPosition(position).withSlot(0));
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


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void stop() {
        pivotMotor1.stopMotor();
    }




}

