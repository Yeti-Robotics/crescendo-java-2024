package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.RobotDataPublisher.RobotDataSubscription;

import java.util.function.Supplier;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX leftKraken;
    private final TalonFX rightKraken;

    private final StatusSignal<Double> leftVelSignal;
    private final StatusSignal<Double> rightVelSignal;

    private static final MotionMagicVelocityVoltage motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0);

    public static class ShooterConstants {
        public static final int SHOOTER_LEFT_MOTOR = 5; //id
        public static final int SHOOTER_RIGHT_MOTOR = 15; //id
        public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentThreshold(55)
                .withSupplyCurrentLimit(65)
                .withSupplyTimeThreshold(0.1)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(65);
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

        public static final int VELOCITY_THRESHOLD = 5;
    }

    public FlywheelSubsystem() {
        leftKraken = new TalonFX(ShooterConstants.SHOOTER_LEFT_MOTOR, Constants.TalonFXConstants.CANIVORE_NAME);
        rightKraken = new TalonFX(ShooterConstants.SHOOTER_RIGHT_MOTOR, Constants.TalonFXConstants.CANIVORE_NAME);

        TalonFXConfigurator rightMotorConfigurator = rightKraken.getConfigurator();
        TalonFXConfigurator leftMotorConfigurator = leftKraken.getConfigurator();

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonConfig.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        talonConfig.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
        talonConfig.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;

        talonConfig.MotionMagic = motionMagicConfigs;

        leftMotorConfigurator.apply(talonConfig);
        rightMotorConfigurator.apply(talonConfig);

        leftKraken.setControl(new Follower(rightKraken.getDeviceID(), false));

        leftVelSignal = leftKraken.getVelocity();
        rightVelSignal = rightKraken.getVelocity();
    }

    @Override
    public void periodic() {
        leftVelSignal.refresh();
        rightVelSignal.refresh();

        SmartDashboard.putNumber("left rps:", leftVelSignal.getValue());
        SmartDashboard.putNumber("right rps:", rightVelSignal.getValue());
    }

    private void stopShooter() {
        rightKraken.stopMotor();
    }

    private void setVelocity(double velocity) {
        rightKraken.setControl(motionMagicVelocityRequest.withVelocity(velocity));
    }

    private double getVelocity() {
        return (leftVelSignal.getValue() + rightVelSignal.getValue()) / 2;
    }

    private boolean reachedVelocity(double vel) {
        return Math.abs(getVelocity() - vel) < ShooterConstants.VELOCITY_THRESHOLD;
    }

    public Command reachVelocity(double vel) {
        return startEnd(() -> setVelocity(vel), this::stopShooter)
                .until(() -> reachedVelocity(vel));
    }

    public Command reachVelocity(Supplier<Double> vel) {
        return runEnd(() -> setVelocity(vel.get()), this::stopShooter)
                .until(() -> reachedVelocity(vel.get()));
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