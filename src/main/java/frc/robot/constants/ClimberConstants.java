package frc.robot.constants;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public final class ClimberConstants {

    public static final int CLIMBER_TALON_1 = 13;
    public static final int CLIMBER_TALON_2 = 14;
    public static final int SERVO_CHANNEL = 6;
    public static final int SERVO_CHANNEL_2 = 7;
    public static final double CLIMB_SPEED = .05;

    public static final SoftwareLimitSwitchConfigs CLIMBER_SOFT_LIMIT_2 = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(-65)
            .withReverseSoftLimitThreshold(0);
    public static final SoftwareLimitSwitchConfigs CLIMBER_SOFT_LIMIT_1 = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(65)
            .withReverseSoftLimitThreshold(0);



}