package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;

import java.nio.file.Path;

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
        TESTING("testing", DEFAULT_CONSTRAINTS);

        public final String name;
        public final PathConstraints initConstraint;
        public final PathConstraints[] pathConstraints;

        AutoModes(String name, PathConstraints initConstraint, PathConstraints... pathConstraints) {
            this.name = name;
            this.initConstraint = initConstraint;
            this.pathConstraints = pathConstraints;
        }
    }}
