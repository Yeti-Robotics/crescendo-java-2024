package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public class ElevatorConstants {
    public static int ELEVATOR_ID = 0;
    public static int ELEVATOR_CAN_ID = 0;
//    public static final double ELEVATOR_P = 0;//set this poportion
//    public static final double ELEVATOR_I = 0;//set this faster
//    public static final double ELEVATOR_D = 0;//set this slower
//    public static final double ELEVATOR_F = 0; //set this kinda fatser
    public static final double ELEVATOR_POSITION_STATUS_FRAME = 0.05;
    public static final double ELEVATOR_VELOCITY_STATUS_FRAME = 0.01;


    public static final CurrentLimitsConfigs ELEVATOR_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(0).withSupplyCurrentLimit(0).withSupplyTimeThreshold(0).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(0);
//REPLACE THE 0's IWTH ACTUAL NUMBERSSSS ^^^^^^ VVVVVVVVV BOTH STATEMNTS ABOVE AND BELOWOWOWOWOWOWOOWWOOW
    public static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(0).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(0);

}
