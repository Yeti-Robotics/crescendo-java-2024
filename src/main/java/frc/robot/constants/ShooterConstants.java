package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.util.ShooterStateData;

public final class ShooterConstants {

    public static final int SHOOTER_LEFT_MOTOR = 15; //id
    public static final int SHOOTER_RIGHT_MOTOR = 5; //id
    public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
    public static final InvertedValue SHOOTER_INVERSION = InvertedValue.CounterClockwise_Positive;

    public static final double SHOOTER_P = 0.11;//0.043315
    public static final double SHOOTER_S = 0.25;//0.043315
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 0.0545 + 7; //+ 4
    public static final double SHOOTER_V = 0.12;
    public static final double SHOOTER_A = 0.01;
    public static final double MOTION_MAGIC_ACCELERATION = 0.1;

    public static final double FLYWHEEL_DIAMETER_IN = 4.0; // inches
    public static final double FLYWHEEL_DIAMETER_M = 0.1016; // meters

    public static final double TOLERANCE_RPM = 10.0;
    public static final double BUMP_FIRE_VEL = 3000 / 60.0;
    public static final double AMP_VEL = 600 / 60.0;
    public static final double TRAP_VEL = 900.0 / 60.0;
    public static final double DEFAULT_VEL = 300 / 60.0;

    public static final double PULLEY_RATIO = 2 / 3;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().
            withKS(SHOOTER_S).
            withKP(SHOOTER_P).
            withKI(SHOOTER_I).
            withKD(SHOOTER_D).
            withKA(MOTION_MAGIC_ACCELERATION).
            withKV(SHOOTER_V);


    public static final int SHOOTER_NEO = 16; //ids i hope pleaseeeee
    public static final int BEAM_BREAK = 0;
    public static final double STAGE_SPEED = -1;

    public static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
        InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
        map.put(0.0, new ShooterStateData(.5, 100));
        map.put(0.4, new ShooterStateData(.5, 100));
        map.put(0.5, new ShooterStateData(.48,100));
        map.put(2.0, new ShooterStateData(0.42,100));
        map.put(2.638109, new ShooterStateData(0.427075, 125));
        map.put(3.0, new ShooterStateData(.44, 125)); //protected shot
        map.put(4.5, new ShooterStateData(0.48,125));
        map.put(5.88, new ShooterStateData(0.42,125));
        return map;
    }
}
