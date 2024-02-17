package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class SparkMaxConstants {
    public static final double PULSES_PER_REV = 4096.0;

    public static final int HIGH_PRIORITY_MS = 20;
    public static final int MEDIUM_PRIORITY_MS = 60;
    public static final int LOW_PRIORITY_MS = 120;
    public static final int NEO_CURRENT_LIM = 30;
    public static final int NEO550_CURRENT_LIM = 20;

    public static final int SPARK_RESOLUTION = 42;
    public static final double COUNTS_PER_DEG = SPARK_RESOLUTION / 360.0;

    public static final double SHOOTER_P = 1;
    public static final double SHOOTER_I = 1;
    public static final double SHOOTER_D =  1;

    public static final double SHOOTER_V = 1;
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(SHOOTER_P).withKI(SHOOTER_I).withKD(SHOOTER_D).withKV(SHOOTER_V);
}