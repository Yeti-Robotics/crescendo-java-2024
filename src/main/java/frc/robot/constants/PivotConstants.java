package frc.robot.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PivotConstants {
    public static final int PIVOT_LIMIT_SWITCH_FORWARD = 7;
    public static final int PIVOT_LIMIT_SWITCH_REVERSE = 6;
    public static final String PIVOT_ONE_MOTOR = "pivotMotor1";
    public static final String PIVOT_TWO_MOTOR = "pivotMotor2";
    public static final int PIVOT_ONE_MOTOR_ID = 29; //placeholder
    public static final String PIVOT_ONE_CANCODER = "pivotEncoder1";
    public static final String PIVOT_TWO_CANCODER = "pivotEncoder2";
    public static final int PIVOT_ONE_CANCODER_ID = 16; //placeholder

    public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final double PIVOT_POSITION_STATUS_FRAME = 0.05;

    public static final double PIVOT_VELOCITY_STATUS_FRAME = 0.01;

    public static final double PIVOT_HOME_POSITION = 0.5;

    public static final double GRAVITY_FEEDFORWARD = 1; //placeholder

    public static final double ANGLE_TOLERANCE = 0.2;

    public static final double PIVOT_P = 50; //1 //350.0 /11.7
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 2; //0 //45.0
    public static final double PIVOT_F = 0;
    public static final double PIVOT_V = 0.12000000149011612; // 65
    public static final double PIVOT_A = 0.009999999776482582; // 0.7
    public static final double PIVOT_G = 0.02; // 0.35
    public static final double PROFILE_V = 0.000000001;
    public static final double PROFILE_A = 0.000000001;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(PIVOT_P).withKI(PIVOT_I).withKD(PIVOT_D).
    withKA(PIVOT_A).withKV(PIVOT_V).withKG(PIVOT_G).withGravityType(GravityTypeValue.Arm_Cosine);

    public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(55).
            withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

    public static final SoftwareLimitSwitchConfigs PIVOT_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(
            .53
    ).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(
            .31
    );

    public static final double MAGNET_OFFSET = 0.433838; //placeholder

    public static final double GEAR_RATIO = 1.0/144.0;

    public enum PivotPositions {
        BUMPFIRE(52),
        ADJUSTABLE_POSITIONS(0); //placeholder
        public final double angle;
        public final double sensorUnits;
        PivotPositions(double angle){
            this.angle = angle;
            this.sensorUnits = angle/GEAR_RATIO * CANCoderConstants.COUNTS_PER_DEG;
        }
    }
}
