package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
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
import frc.robot.constants.ConfiguratorConstants;

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
    private PivotPositions pivotPositions = PivotPositions.BUMPFIRE;
    public static final int PIVOT_LIMIT_SWITCH_FORWARD = 7;
    public static final int PIVOT_LIMIT_SWITCH_REVERSE = 6;
    public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double PIVOT_POSITION_STATUS_FRAME = 0.05;
    public static final double PIVOT_VELOCITY_STATUS_FRAME = 0.01;
    public static final double PIVOT_HOME_POSITION = 0.5;
    public static final double PIVOT_P = 50; //1 //350.0 /11.7
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 2; //0 //45.0
    public static final double PIVOT_V = 0.12000000149011612; // 65
    public static final double PIVOT_A = 0.009999999776482582; // 0.7
    public static final double PIVOT_G = 0.02; // 0.35
    public static final double PROFILE_V = 0.000000001;
    public static final double PROFILE_A = 0.000000001;
    public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(55).
            withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

    public static final SoftwareLimitSwitchConfigs PIVOT_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(
            .53
    ).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(
            .31
    );

    public static final double MAGNET_OFFSET = 0.433838; //placeholder

    public static final double GEAR_RATIO = 1.0/144.0;

    public enum PivotPositions {
        BUMPFIRE(52);
        public final double angle;
        public final double sensorUnits;
        PivotPositions(double angle){
            this.angle = angle;
            this.sensorUnits = angle/GEAR_RATIO * ConfiguratorConstants.COUNTS_PER_DEG;
        }
    }

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(PIVOT_P).withKI(PIVOT_I).withKD(PIVOT_D).
            withKA(PIVOT_A).withKV(PIVOT_V).withKG(PIVOT_G).withGravityType(GravityTypeValue.Arm_Cosine);


    public PivotSubsystem() {
        reverseLimitSwitch = new DigitalInput(PIVOT_LIMIT_SWITCH_REVERSE);
        forwardLimitSwitch = new DigitalInput(PIVOT_LIMIT_SWITCH_FORWARD);
        pivotMotor1 = new TalonFX(ConfiguratorConstants.PIVOT_ONE_MOTOR_ID, ConfiguratorConstants.CANIVORE_NAME);
        pivotEncoder1 = new CANcoder(ConfiguratorConstants.PIVOT_ONE_CANCODER_ID, ConfiguratorConstants.CANIVORE_NAME);
        pivotMotor1.setInverted(true);
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);

        var pivotMotor1Configurator = pivotMotor1.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder1.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfiguration.MotorOutput.NeutralMode = PIVOT_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 83.79;
        talonFXConfiguration.CurrentLimits = PIVOT_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = PIVOT_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 2;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = PROFILE_A;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = PROFILE_V;

        pivotMotor1.getRotorVelocity().waitForUpdate(PIVOT_VELOCITY_STATUS_FRAME);
        pivotMotor1.getRotorPosition().waitForUpdate(PIVOT_POSITION_STATUS_FRAME);
        pivotEncoder1.getAbsolutePosition().waitForUpdate(0.01);


        pivotMotor1Configurator.apply(talonFXConfiguration);

        var pivotEncoder1Configurator = pivotEncoder1.getConfigurator();

        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
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
        return pivotMotor1.getRotorPosition().getValue() / ConfiguratorConstants.COUNTS_PER_DEG * GEAR_RATIO;
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

