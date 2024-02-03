package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ArmConstants {

    public static final int ARM_KRAKEN_ID = 2;
    public static final int ARM_CANCODER_ID = 1;
    public static final int BEAM_BREAK_PORT = 1;

    public static final InvertedValue ARM_INVERSION = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double ARM_POSITION_STATUS_FRAME = 0.05;
    public static final double ARM_VELOCITY_STATUS_FRAME = 0.01;

    public static final double GRAVITY_FEEDFORWARD = 0.05;

    public static final double ANGLE_TOLERANCE = 5;

    public static final double ARM_P = 0;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_F = 0;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ARM_P).withKI(ARM_I).withKD(ARM_D).withGravityType(GravityTypeValue.Arm_Cosine);
    public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

    public static final SoftwareLimitSwitchConfigs ARM_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(
            55 //placeholder
    ).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(
            65 //placeholder
    );
    public static final double MAGNET_OFFSET = 0; //placeholder



    public static final double GEAR_RATIO = 1.0 / (50.463 / 12.0);

    public enum ArmPositions {
        DOWN(-15),
        STOWED(90);

        public final double angle;
        public final double sensorUnits;

        ArmPositions(double angle) {
            this.angle = angle;
            this.sensorUnits = angle / GEAR_RATIO * CANCoderConstants.COUNTS_PER_DEG;
        }
    }
}