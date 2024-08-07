package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.TalonFXConstants;

import static edu.wpi.first.units.Units.Seconds;

public class PivotSubsystem extends SubsystemBase {

    private static final Measure<Velocity<Voltage>> sysIdRampRate =
            edu.wpi.first.units.Units.Volts.of(1).per(Seconds.of(1));
    private static final Measure<Voltage> sysIdStepAmps = edu.wpi.first.units.Units.Volts.of(7);
    public final TalonFX pivotMotor1;
    public final CANcoder pivotEncoder1;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    // SysID Setup
    private final SysIdRoutine sysIdRoutine;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;
    public final Trigger anyLimitSwitchPressed = new Trigger(() -> getForwardLimitSwitch() || getReverseLimitSwitch());
    private PivotConstants.PivotPositions pivotPositions = PivotConstants.PivotPositions.BUMPFIRE;

    public PivotSubsystem() {
        reverseLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_REVERSE);
        forwardLimitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH_FORWARD);
        pivotMotor1 = new TalonFX(PivotConstants.PIVOT_ONE_MOTOR_ID, TalonFXConstants.CANIVORE_NAME);
        pivotEncoder1 = new CANcoder(PivotConstants.PIVOT_ONE_CANCODER_ID, TalonFXConstants.CANIVORE_NAME);
        pivotMotor1.setInverted(true);
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);

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
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 2;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = PivotConstants.PROFILE_A;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = PivotConstants.PROFILE_V;

        pivotMotor1.getRotorVelocity().waitForUpdate(PivotConstants.PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor1.getRotorPosition().waitForUpdate(PivotConstants.PIVOT_POSITION_STATUS_FRAME);
        pivotEncoder1.getAbsolutePosition().waitForUpdate(0.01);


        pivotMotor1Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder1.getConfigurator();

        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = PivotConstants.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
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
        SmartDashboard.putData("Pivot kraken", pivotMotor1);
        SmartDashboard.putData("Pivot encoder", pivotEncoder1);
        SmartDashboard.putData("Forward limit switch pivot", forwardLimitSwitch);
        SmartDashboard.putData("Reverse limit switch pivot", reverseLimitSwitch);
    }

    public void setPivotPosition(double position) {
        MotionMagicVoltage motionMagic = new MotionMagicVoltage(
                position, true, 0, 0,
                false, false, false);
        // todo: overridebreakdurneutral = false

        System.out.print("Pivot position: ");
        System.out.println(position);
        System.out.println(motionMagic.Position);
        pivotMotor1.setControl(motionMagic.withPosition(position).
                withLimitForwardMotion(getForwardLimitSwitch())
                .withLimitReverseMotion(getReverseLimitSwitch()).
                withSlot(0).withUpdateFreqHz(200));
        SmartDashboard.putNumber("pivot position setpoint:", position);
    }

    public double getAngle() {
        return pivotMotor1.getRotorPosition().getValue() / CANCoderConstants.COUNTS_PER_DEG * PivotConstants.GEAR_RATIO;
    }

    public double getEncAngle() {
        return pivotEncoder1.getAbsolutePosition().getValue();
    }

    public void moveUp(double speed) {
        if (!getForwardLimitSwitch()) {
            pivotMotor1.set(Math.abs(speed));
        }
    }

    public void moveDown(double speed) {
        if (!getReverseLimitSwitch()) {
            pivotMotor1.set(-Math.abs(speed));
        }
    }

    public boolean getForwardLimitSwitch() {
        return !forwardLimitSwitch.get();
    }

    public boolean getReverseLimitSwitch() {
        return !reverseLimitSwitch.get();
    }

    public void stop() {
        pivotMotor1.stopMotor();
    }


}

