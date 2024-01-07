package frc.robot.constants;

public final class SparkMaxConstants {
    public static final double PULSES_PER_REV = 4096.0;

    public static final int HIGH_PRIORITY_MS = 20;
    public static final int MEDIUM_PRIORITY_MS = 60;
    public static final int LOW_PRIORITY_MS = 120;
    public static final int NEO_CURRENT_LIM = 30;
    public static final int NEO550_CURRENT_LIM = 20;

    public static final int SPARK_RESOLUTION = 42;
    public static final double COUNTS_PER_DEG = SPARK_RESOLUTION / 360.0;
}