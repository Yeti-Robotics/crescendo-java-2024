package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
    public static int STAGES = 3;
    public static final double STAGE_EXTENSION = 0; //PLACEHOLDER
    public static final double MAX_EXTENSION = STAGE_EXTENSION * STAGES; //PLACEHOLDER
    public static final double ELEVATOR_DISTANCE_PER_PULSE = 1; //PLACEHOLDER
    public static int ELEVATOR_ID = 10;
    public static int ELEVATOR_CAN_ID = 0;
    public static final double ELEVATOR_P = 20;//set this poportion
    public static final double ELEVATOR_I = 0;//set this faster
    public static final double ELEVATOR_D = 0. ;//set this slower
    public static final double ELEVATOR_F = 0; //set this kinda fatser
    public static final double ELEVATOR_G = 0.0;//set this kinda fatser
    public static final double ELEVATOR_A = 0.05; //set this kinda fatser
    public static final double ELEVATOR_V = 8.0; //set this kinda fatser
    public static final double PROFILE_V = 2.5; //set this kinda fatser
    public static final double PROFILE_A = .5; //set this kinda fatser

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D)
            .withKA(ELEVATOR_A).withKV(ELEVATOR_V).withKG(ELEVATOR_G).withGravityType(GravityTypeValue.Elevator_Static);

    public static final double ELEVATOR_POSITION_STATUS_FRAME = 0.05;
    public static final double ELEVATOR_VELOCITY_STATUS_FRAME = 0.01;
    public static final double ELEVATOR_GEAR_RATIO = 1;


    public static final CurrentLimitsConfigs ELEVATOR_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(65).withSupplyCurrentLimit(75).withSupplyTimeThreshold(1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(75);
    public static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);

    public enum ElevatorPositions {
        DOWN(0),
        AMP(8),
        TRAP(15);
        public final double distanceEl;
        public final double sensorUnitsEl;
        ElevatorPositions(double distance) {
            this.distanceEl = distance;
            this.sensorUnitsEl = (distance / STAGES) / ELEVATOR_DISTANCE_PER_PULSE;
        }
    }
}
