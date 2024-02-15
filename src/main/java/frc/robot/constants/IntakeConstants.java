package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
    public static final int INTAKE_KRAKEN_ID = 19;

    public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
    public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01;

}