package frc.robot.constants;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public final class ClimberConstants {

    public static final int CLIMBER_TALON_1 = 13;
    public static final int CLIMBER_TALON_2 = 14;
    public static final int CLIMBER_ENCODER_1 = 13;
    public static final int CLIMBER_ENCODER_2 = 14;
    public static final int SERVO_CHANNEL = 9;
    public static final int SERVO_CHANNEL_2 = 0;
    public static final double CLIMB_SPEED = .05;

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


    public static final double WPILIB_MIN_SERVO_ANGLE = 0.0; //degrees
    public static final double WPILIB_MAX_SERVO_ANGLE = 360.0; //degrees
    public static final double TIME_TO_SERVO_FULL_EXTENSION = 3.48; //Avg time to move from retract to extend
    public static final double PERCENT_PER_SECOND = 1.00 / TIME_TO_SERVO_FULL_EXTENSION;
    public static final double DEGREES_PER_SECOND = (WPILIB_MAX_SERVO_ANGLE - WPILIB_MIN_SERVO_ANGLE)
            * PERCENT_PER_SECOND;

    public static final double MAX_POSITION = 1.0; //percent servo travel to max BRAKE position
    public static final double MIN_POSITION = 0.0; //percent servo travel to min BRAKE position

    public static final double MAX_SERVO_PWM = 2.0; //ms
    public static final double MIN_SERVO_PWM = 1.0; //ms
    public static final double SERVO_RANGE = MAX_SERVO_PWM - MIN_SERVO_PWM;
    public static final double CENTER_SERVO_PWM = 1.5; //ms
    public static final double SERVO_DEADBAND = 0.0; //ms - no deadband

    // pwm values in ms for the max and min angles of the shooter BRAKE
    public static final double BRAKE_MAX_PWM = MIN_SERVO_PWM + (SERVO_RANGE * MAX_POSITION);
    public static final double BRAKE_MIN_PWM = MIN_SERVO_PWM + (SERVO_RANGE * MIN_POSITION);


}