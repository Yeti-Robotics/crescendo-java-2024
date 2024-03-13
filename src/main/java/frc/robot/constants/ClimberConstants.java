package frc.robot.constants;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public final class ClimberConstants {

    public static final int CLIMBER_TALON_1 = 13;
    public static final int CLIMBER_TALON_2 = 14;
    public static final int CLIMBER_ENCODER_1 = 13;
    public static final int CLIMBER_ENCODER_2 = 14;
    public static final int SERVO_CHANNEL = 9;
    public static final int SERVO_CHANNEL_2 = 0;
    public static final double CLIMB_SPEED = .6;

    public static final SoftwareLimitSwitchConfigs CLIMBER_SOFT_LIMIT_2 = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(-65)
            .withReverseSoftLimitThreshold(0);
    public static final SoftwareLimitSwitchConfigs CLIMBER_SOFT_LIMIT_1 = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(65)
            .withReverseSoftLimitThreshold(0);



    public static final int BRAKE_MAX_POSITION = 1;
    private static final int BRAKE_MIN_POSITION = 0;

    //SERVO Parameters from https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
    public static final int MAX_SERVO_PWM = 2; //ms
    public static final int MIN_SERVO_PWM = 1; //ms
    public static final int SERVO_RANGE = MAX_SERVO_PWM - MIN_SERVO_PWM;
    public static final int CENTER_SERVO_PWM = 2; //ms
    public static final int SERVO_DEADBAND = 0; //ms - no deadband
    public static final int BRAKE_MAX_PWM = MIN_SERVO_PWM + (SERVO_RANGE * BRAKE_MAX_POSITION);
    public static final int BRAKE_MIN_PWM = MIN_SERVO_PWM + (SERVO_RANGE * BRAKE_MIN_POSITION);


}