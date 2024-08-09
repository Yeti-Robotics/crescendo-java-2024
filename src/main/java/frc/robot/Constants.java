package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ShooterStateData;

import java.io.IOException;
import java.util.Map;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

public class Constants {

    public class FieldConstants{
        public static final double fieldLength = Units.inchesToMeters(651.223);

        public static final class Speaker {
            /**
             * Center of the speaker opening (blue alliance)
             */
            public static Translation3d topRightSpeaker =
                    new Translation3d(
                            Units.inchesToMeters(18.055),
                            Units.inchesToMeters(238.815),
                            Units.inchesToMeters(83.091));
            public static Translation3d bottomLeftSpeaker =
                    new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
            public static Translation3d centerSpeakerOpening =
                    bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
            public static final AprilTagFieldLayout aprilTags;
            static {
                try {
                    aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public class CANCoderConstants{
        public static final int COUNTS_PER_REV = 4096;
        public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;
    }

    public class OIConstants{
        public static final Map<Integer, ControllerType> CONTROLLERS = Map.of(
                0, ControllerType.CUSTOM, 1, ControllerType.XBOX
        );

        public static final int CONTROLLER_COUNT = CONTROLLERS.size(); //placehold


        public enum ControllerType {
            CUSTOM,
            XBOX
        }
    }

    public class TalonFXConstants{
        public static final boolean TALON_FUTURE_PROOF = true;
        public static final String CANIVORE_NAME = "canivoreBus";
    }

    public class ArmConstants{
        public static final int ARM_KRAKEN_ID = 21;
        public static final int ARM_CANCODER_ID = 5;

        public static final InvertedValue ARM_INVERSION = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double ARM_POSITION_STATUS_FRAME = 0.05;
        public static final double ARM_VELOCITY_STATUS_FRAME = 0.01;
        public static final double ARM_HANDOFF_POSITION = 0.03;

        public static final double GRAVITY_FEEDFORWARD = 0.05;

        public static final double ANGLE_TOLERANCE = 5;

        public static final double ARM_P = 0;
        public static final double ARM_I = 0;
        public static final double ARM_D = 0;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ARM_P).withKI(ARM_I).withKD(ARM_D).withGravityType(GravityTypeValue.Arm_Cosine);
        public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

        public static final SoftwareLimitSwitchConfigs ARM_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().
                withForwardSoftLimitEnable
                        (true).
                withForwardSoftLimitThreshold(
                        0.0195 //placeholder
                ).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(
                        65 //placeholder
                );
        public static final double MAGNET_OFFSET = -1;

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

    public class DriveConstants{
        public static final double SUPPLY_CURRENT_LIMIT = 60;
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD = 65;
        public static final double SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD = 0.1;

        public static final double PEAK_FORWARD_VOLTAGE = 12.0;
        public static final double PEAK_REVERSE_VOLTAGE = -12.0;

        public static final double SWERVE_X_REDUCTION = 1.0 / 6.75;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //0.1016

        public static final double MaFxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SWERVE_X_REDUCTION * WHEEL_DIAMETER * Math.PI; //placeholder

        private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER
        private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER

        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(frc.robot.Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                                frc.robot.Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                        // Front right
                        new Translation2d(frc.robot.Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                                -frc.robot.Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                        // Back left
                        new Translation2d(-frc.robot.Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                                frc.robot.Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                        // Back right
                        new Translation2d(-frc.robot.Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                                -frc.robot.Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
                );
    }

    public class ElevatorConstants{
        public static int STAGES = 3;
        public static final double ELEVATOR_DISTANCE_PER_PULSE = 1; //PLACEHOLDER
        public static int ELEVATOR_ID = 10;
        public static final double ELEVATOR_P = 45;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0.;
        public static final double ELEVATOR_G = 0.0;
        public static final double ELEVATOR_A = 0.05;
        public static final double ELEVATOR_V = 8.0;
        public static final double PROFILE_V = 0.2;
        public static final double PROFILE_A = .5;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D)
                .withKA(ELEVATOR_A).withKV(ELEVATOR_V).withKG(ELEVATOR_G).withGravityType(GravityTypeValue.Elevator_Static);

        public static final double ELEVATOR_POSITION_STATUS_FRAME = 0.05;
        public static final double ELEVATOR_VELOCITY_STATUS_FRAME = 0.01;

        public static final CurrentLimitsConfigs ELEVATOR_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(65).withSupplyCurrentLimit(75).withSupplyTimeThreshold(1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(75);
        public static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);

        public enum ElevatorPositions {
            DOWN(0),
            AMP(18);
            public final double distanceEl;
            public final double sensorUnitsEl;
            ElevatorPositions(double distance) {
                this.distanceEl = distance;
                this.sensorUnitsEl = (distance / STAGES) / ELEVATOR_DISTANCE_PER_PULSE;
            }
            }
    }

    public class IntakeConstants
    {
        public static final int INTAKE_KRAKEN_ID = 8;

        public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
        public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01;
    }

    public class PivotConstants{
        public static final int PIVOT_LIMIT_SWITCH_FORWARD = 7;
        public static final int PIVOT_LIMIT_SWITCH_REVERSE = 6;
        public static final int PIVOT_ONE_MOTOR_ID = 29; //placeholder
        public static final int PIVOT_ONE_CANCODER_ID = 16; //placeholder

        public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final double PIVOT_POSITION_STATUS_FRAME = 0.05;

        public static final double PIVOT_P = 50; //1 //350.0 /11.7
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 2; //0 //45.0
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

        public enum PivotPosition {
            CUSTOM(-1) {
                @Override
                public double getPosition() {
                    if (super.getPosition() == -1) {
                        throw new IllegalArgumentException("Custom pivot position was not set");
                    }

                    return super.getPosition();
                }
            },
            HANDOFF(0.5);
            //placeholder
            private double position;

            PivotPosition(double position){
                this.position = position;
            }

            public double getPosition() {
                return position;
            }

            public static PivotPosition CUSTOM(double position) {
                PivotPosition pivotPosition = PivotPosition.CUSTOM;
                pivotPosition.position = position;
                return pivotPosition;
            }
        }
    }

    public class ShooterConstants{
        public static final int SHOOTER_LEFT_MOTOR = 5; //id
        public static final int SHOOTER_RIGHT_MOTOR = 15; //id
        public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
        public static final InvertedValue SHOOTER_INVERSION = InvertedValue.CounterClockwise_Positive;

        public static final double SHOOTER_P = 0.11;//0.043315
        public static final double SHOOTER_S = 0.25;//0.043315
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_V = 0.12;
        public static final double MOTION_MAGIC_ACCELERATION = 0.1;
        public static final double SHOOTER_STATUS_FRAME_SECONDS = 0.01;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().
                withKS(SHOOTER_S).
                withKP(SHOOTER_P).
                withKI(SHOOTER_I).
                withKD(SHOOTER_D).
                withKA(MOTION_MAGIC_ACCELERATION).
                withKV(SHOOTER_V);


        public static final int SHOOTER_NEO = 16;
        public static final int BEAM_BREAK = 0;

        public static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
            InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
            // TODO: decrease angles by aroun 0.05 to tune
            map.put(1.375, new ShooterStateData(0.5, 125));
            map.put(1.7, new ShooterStateData(.485, 125));
            map.put(2.0, new ShooterStateData(0.478, 125));
            map.put(2.3, new ShooterStateData(0.47, 125));
            map.put(2.65, new ShooterStateData(.465, 125));
            map.put(2.8, new ShooterStateData(.4625, 125));
            map.put(3.0, new ShooterStateData(0.46, 125));
            map.put(3.8, new ShooterStateData(0.443, 125));
            return map;
        }
    }

    public class VisionConstants{
        public static final String LIMELIGHT_NAME = "limelight";
        public static final double BLINK_DELAY_SECONDS = 0.125;
        public static final int NUM_BLINKS = 4;
    }
}
