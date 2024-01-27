package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public final class ShooterConstants {

    public static final int SHOOTER_LEFT_MOTOR = 15;
    public static final int     SHOOTER_RIGHT_MOTOR = 16;
    public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
    public static final InvertedValue SHOOTER_INVERSION = InvertedValue.CounterClockwise_Positive;

    public static final double SHOOTER_P = 0.1;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 0.0545 + 5;
    public static final double SHOOTER_V = 0.45887 + 0.3;
    public static final double SHOOTER_A = .05;

    public static final double FLYWHEEL_DIAMETER_IN = 4.0; // inches
    public static final double FLYWHEEL_DIAMETER_M = 0.1016; // meters

    public static final double TOLERANCE_RPM = 10.0;
    public static final double BUMP_FIRE_VEL = 3000 / 60.0;
    public static final double AMP_VEL = 600 / 60.0;
    public static final double TRAP_VEL = 900.0 / 60.0;
    public static final double DEFAULT_VEL = 300 / 60.0;

    public static final double PULLEY_RATIO = 2 / 3;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(SHOOTER_P).withKI(SHOOTER_I).withKD(SHOOTER_D).withKA(SHOOTER_A).withKV(SHOOTER_V);

}