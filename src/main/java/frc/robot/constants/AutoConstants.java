package frc.robot.constants;


import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;

public final class AutoConstants {

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
    public static final double MAX_ACCEL = TunerConstants.kSpeedAt12VoltsMps;
    public static final double MAX_THETA_VELOCITY =  MAX_SPEED / Math.hypot(22.25 / 2, 22.25 / 2);
    public static final double MAX_THETA_ACCEL = MAX_SPEED / Math.hypot(22.25 / 2, 22.25 / 2);
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
            MAX_SPEED,
            MAX_ACCEL,
            MAX_THETA_VELOCITY,
            MAX_THETA_ACCEL);

    public enum AutoModes {
        AMP_4_THREE_PIECE("amp4ThreePiece", DEFAULT_CONSTRAINTS),
        MID_3_THREE_PIECE("mid3ThreePiece",DEFAULT_CONSTRAINTS),
        MID_3_TWO_PIECE("mid3TwoPiece", DEFAULT_CONSTRAINTS),
        AMP_4_TWO_PIECE("amp4TwoPiece", DEFAULT_CONSTRAINTS),
        MID_SUB_TWO_PIECE("midSubTwoPiece", DEFAULT_CONSTRAINTS),
        MID_SUB_THREE_PIECE("midSubThreePiece", DEFAULT_CONSTRAINTS),
        MID_SUB_FOUR_PIECE("midSubFourPiece", DEFAULT_CONSTRAINTS),
        MID_3_ONE_PIECE("mid3OnePiece", DEFAULT_CONSTRAINTS);

        public final String name;
        public final PathConstraints initConstraint;
        public final PathConstraints[] pathConstraints;

        AutoModes(String name, PathConstraints initConstraint, PathConstraints... pathConstraints) {
            this.name = name;
            this.initConstraint = initConstraint;
            this.pathConstraints = pathConstraints;
        }
    }}