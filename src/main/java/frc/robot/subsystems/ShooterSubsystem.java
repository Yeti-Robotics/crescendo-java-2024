package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConfiguratorConstants;
import frc.robot.util.ShooterStateData;

public class ShooterSubsystem extends SubsystemBase {

    public static ShooterModes shooterModes;
    public static ShooterStatus shooterStatus;
    public static double velocity = 0;
    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    private final TalonFX neo;
    private final DigitalInput beamBreak;
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
    public static final InvertedValue SHOOTER_INVERSION = InvertedValue.CounterClockwise_Positive;

    public static final double SHOOTER_P = 0.11;//0.043315
    public static final double SHOOTER_S = 0.25;//0.043315
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_V = 0.12;
    public static final double MOTION_MAGIC_ACCELERATION = 0.1;


    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().
            withKS(SHOOTER_S).
            withKP(SHOOTER_P).
            withKI(SHOOTER_I).
            withKD(SHOOTER_D).
            withKA(MOTION_MAGIC_ACCELERATION).
            withKV(SHOOTER_V);


    public static final int SHOOTER_NEO = 16; //ids i hope pleaseeeee
    public static final int BEAM_BREAK = 0;

    public static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
        InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
        // TODO: decrease angles by aroun 0.05 to tune
        map.put(1.375, new ShooterStateData(0.5, 125));
        map.put(1.7, new ShooterStateData(.485, 125));
        map.put(2.0, new ShooterStateData(0.478, 125));
        map.put(2.3, new ShooterStateData(0.47, 125));
        map.put(2.65, new ShooterStateData(.465, 125));
        map.put(2.8, new ShooterStateData(.4625, 125));
        map.put(3.0, new ShooterStateData(0.46, 125));
        map.put(3.8, new ShooterStateData(0.443, 125));
        return map;
    }

    public ShooterSubsystem() {
        leftKraken = new TalonFX(ConfiguratorConstants.SHOOTER_LEFT_MOTOR, ConfiguratorConstants.CANIVORE_NAME);
        rightKraken = new TalonFX(ConfiguratorConstants.SHOOTER_RIGHT_MOTOR, ConfiguratorConstants.CANIVORE_NAME);
        var rightMotorConfigurator = rightKraken.getConfigurator();
        var leftMotorConfigurator = leftKraken.getConfigurator();
        var rightMotorConfiguration = new TalonFXConfiguration();

        rightMotorConfiguration.MotorOutput.Inverted = SHOOTER_INVERSION;
        rightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfiguration.CurrentLimits = SHOOTER_CURRENT_LIMIT;
        rightMotorConfiguration.Slot0 = SLOT_0_CONFIGS;
        leftKraken.setControl(new Follower(rightKraken.getDeviceID(), false));
        shooterStatus = ShooterStatus.OFF;
        shooterModes = ShooterModes.TRAP;


        neo = new TalonFX(SHOOTER_NEO, "canivoreBus");

        beamBreak = new DigitalInput(BEAM_BREAK);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        var motionMagicConfigs = rightMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        rightMotorConfigurator.apply(rightMotorConfiguration);
        leftMotorConfigurator.apply(rightMotorConfiguration);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("shooter beam break", beamBreak);
        SmartDashboard.putNumber("left rps:", leftKraken.getVelocity().getValue());
        SmartDashboard.putNumber("right rps:", rightKraken.getVelocity().getValue());
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    public void spinNeo() {
        neo.set(-1);
    }

    public void spinFeeder(double speed) {
        neo.set(speed);
    }

    public void stopNeo() {
        neo.stopMotor();
    }

    public void shootFlywheel(double speed) {
        rightKraken.set(speed);
        leftKraken.set(speed);
        neo.set(0.9); //
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void stopFlywheel() {
        rightKraken.stopMotor();
        leftKraken.stopMotor();
        neo.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }

    public void setVelocity(double vel) {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
    }

    public void bumpFire() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(60));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(100));
    }

    public Command bumpFireCmd() {
        return runOnce(this::bumpFire);
    }

    public Command spinNeoCmd() {
        return Commands.startEnd(this::spinNeo, this::stopFlywheel);
    }

    public Command setVelocityCmd(double vel) {
        return startEnd(() -> setVelocity(vel), this::stopFlywheel);
    }

    public Command setVelocityInstantCommand(double vel) {
        return runOnce(() -> setVelocity(vel));
    }

    public Command spinFeederCmd(double vel) {
        return startEnd(() -> spinFeeder(vel), this::stopNeo);
    }

    public Command shootTrapCmd() {
        return startEnd(this::shootTrap, this::stopFlywheel);
    }

    public void shootTrap() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(35));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(80));
    }

    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }

}
