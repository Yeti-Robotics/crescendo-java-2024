package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {
    /**
     * Max velocity in meters per second
     */
    public static final double MAX_VELOCITY = DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.75;
    /**
     * Max acceleration in meters per second squared
     */
    public static final double MAX_ACCEL = MAX_VELOCITY * 0.75;
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(MAX_VELOCITY, MAX_ACCEL, Math.toRadians(720),Math.toRadians(720) );
    public static final PathConstraints ALIGNMENT_CONSTRAINTS = new PathConstraints(3.0, 3.0, Math.toRadians(720), Math.toRadians(720));

    public static final double TRANSLATION_P = 3.0; //2.9, 2.15
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.3;
    public static final double THETA_CONTROLLER_P = 3.3; //3
    public static final double THETA_CONTROLLER_I = 0.00; //3
    public static final double THETA_CONTROLLER_D = 0.3;

    public static final PIDConstants TRANSLATION_CONTROLLER = new PIDConstants(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
    public static final PIDConstants THETA_CONTROLLER = new PIDConstants(THETA_CONTROLLER_P, THETA_CONTROLLER_I, THETA_CONTROLLER_D);
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = //
            new TrapezoidProfile.Constraints(
                    DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    public static final double PITCH_SET_POINT = 0.0;
    public static final double PITCH_P = 0.02; //0.02
    public static final double PITCH_I = 0.00;
    public static final double PITCH_D = 0.001; //0.00579 0.001
    public static final double PITCH_TOLERANCE = 1.0;

    public static final double CENTER_OFFSET = Units.inchesToMeters(DriveConstants.FRAME_PERIMETER / 2.0 + 15.0); // 0.4445

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
    }

    public enum ALIGNMENT_POSITION {
        LEFT_DOUBLE_STATION(-CENTER_OFFSET, 1.1, 0.0, 0.0),
        RIGHT_DOUBLE_STATION(-CENTER_OFFSET, -0.9, 0.0, 0.0),
        SINGLE_STATION(-1.72, CENTER_OFFSET, 90.0, 90),
        LEFT(CENTER_OFFSET, -0.53, 180.0, -180.0),
        MIDDLE(CENTER_OFFSET, 0.0, 180.0, 180.0),
        RIGHT(CENTER_OFFSET, 0.53, 180.0, 180.0);

        public final Pose2d offset;
        public final Rotation2d heading;

        ALIGNMENT_POSITION(double x, double y, double heading, double rotation) {
            this.offset = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
            this.heading = Rotation2d.fromDegrees(heading);
        }
    }
}