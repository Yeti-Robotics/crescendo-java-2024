package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

public class ConfiguratorConstants {

    public static final int ARM_KRAKEN_ID = 21;
    public static final int ARM_CANCODER_ID = 5;

    public static int ELEVATOR_ID = 10;

    public static final int INTAKE_KRAKEN_ID = 8;

    public static final int PIVOT_ONE_MOTOR_ID = 29; //placeholder
    public static final int PIVOT_ONE_CANCODER_ID = 16; //placeholder

    public static final int SHOOTER_LEFT_MOTOR = 5; //id
    public static final int SHOOTER_RIGHT_MOTOR = 15; //id

    public static final int COUNTS_PER_REV = 4096;
    public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

    public static final boolean TALON_FUTURE_PROOF = true;

    public static final String CANIVORE_NAME = "canivoreBus";

    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
    public static final double speakerPose = fieldWidth - Units.inchesToMeters(104.0);
    public static final double robotHeight = Units.feetToMeters(2.333);
    public static final double speakerHeightRelativeToBot = Speaker.centerSpeakerOpening.getY()- robotHeight;
    public static final Translation2d ampCenter =
            new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));
    /**
     * Staging locations for each note
     */
    public static final class StagingLocations {
        public static final double centerlineX = fieldLength / 2.0;
        // need to update
        public static final double centerlineFirstY = Units.inchesToMeters(29.638);
        public static final double centerlineSeparationY = Units.inchesToMeters(66);
        public static final double spikeX = Units.inchesToMeters(114);
        // need
        public static final double spikeFirstY = Units.inchesToMeters(161.638);
        public static final double spikeSeparationY = Units.inchesToMeters(57);
        public static final Translation2d[] centerlineTranslations = new Translation2d[5];
        public static final Translation2d[] spikeTranslations = new Translation2d[3];
        static {
            for (int i = 0; i < centerlineTranslations.length; i++) {
                centerlineTranslations[i] =
                        new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
            }
        }
        static {
            for (int i = 0; i < spikeTranslations.length; i++) {
                spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
            }
        }
    }
    /**
     * Each corner of the speaker *
     */
    public static final class Speaker {
        /**
         * Center of the speaker opening (blue alliance)
         */
        public static Translation3d topRightSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(238.815),
                        Units.inchesToMeters(83.091));
        public static Translation3d topLeftSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(197.765),
                        Units.inchesToMeters(83.091));
        public static Translation3d bottomRightSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
        public static Translation3d bottomLeftSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
        public static Translation3d centerSpeakerOpening =
                bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
        public static final double aprilTagWidth = Units.inchesToMeters(6.50);
        public static final AprilTagFieldLayout aprilTags;
        static {
            try {
                aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public static AprilTagFieldLayout aprilTagLayout;
    public static List<Pose2d> aprilTagPoses = new ArrayList<Pose2d>(12);
    public static List<Pose2d> allianceAprilTags = new ArrayList<Pose2d>(6);
    public static Pose2d ampTag = new Pose2d(ampCenter, Rotation2d.fromDegrees(0));
    public static List<Pose2d> opposingAllianceAprilTags = new ArrayList<Pose2d>(6);
    static{
        try {
            aprilTagLayout = k2024Crescendo.loadAprilTagLayoutField();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    }
    public static void updateAprilTagTranslations() {
        aprilTagPoses.clear();
        allianceAprilTags.clear();
        opposingAllianceAprilTags.clear();
        for(int i = 0; i < aprilTagLayout.getTags().size(); i++) {
            aprilTagPoses.add(i, aprilTagLayout.getTagPose(i + 1).get().toPose2d());
        }

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            ampTag = new Pose2d(ampCenter, Rotation2d.fromDegrees(0));
            allianceAprilTags.addAll(aprilTagPoses.subList(1,8));
            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(9,16));
        } else {
            ampTag = new Pose2d(ampCenter.getX(), 0, Rotation2d.fromDegrees(0));
            allianceAprilTags.addAll(aprilTagPoses.subList(9,16));
            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(1,8));

        }
    }

    public static final Map<Integer, ControllerType> CONTROLLERS = Map.of(
            0, ControllerType.CUSTOM, 1, ControllerType.XBOX
    );

    public static final int CONTROLLER_COUNT = CONTROLLERS.size(); //placehold
    public static final double DEADBAND = .05; //placehold
    public static final String Translational_XSupplier = "translationXSupplier";
    public static final String Translational_YSupplier = "translationYSupplier";
    public static final String THETA_SUPPLIER = "thetaSupplier";

    public enum ControllerType {
        CUSTOM,
        XBOX
    }

    public enum ButtonMode {
        PRIMARY
    }

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 6.37;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.8181818181818183;

    private static final double kDriveGearRatio = 5.011363636363637;
    private static final double kSteerGearRatio = 13.371428571428572;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "canivoreBus";
    private static final int kPigeonId = 1;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 9;
    private static final int kFrontLeftSteerMotorId = 4;
    private static final int kFrontLeftEncoderId = 1;
    private static final double kFrontLeftEncoderOffset = -0.238037;

    private static final double kFrontLeftXPosInches = 11.125;
    private static final double kFrontLeftYPosInches = 11.125;

    // Front Right
    private static final int kFrontRightDriveMotorId = 11;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 3;
    private static final double kFrontRightEncoderOffset = -0.393311;

    private static final double kFrontRightXPosInches = 11.125;
    private static final double kFrontRightYPosInches = -11.125;

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 0;
    private static final int kBackLeftEncoderId = 2;
    private static final double kBackLeftEncoderOffset = 0.498535;

    private static final double kBackLeftXPosInches = -11.125;
    private static final double kBackLeftYPosInches = 11.125;

    // Back Right
    private static final int kBackRightDriveMotorId = 6;
    private static final int kBackRightSteerMotorId = 2;
    private static final int kBackRightEncoderId = 0;
    private static final double kBackRightEncoderOffset = -0.019287;

    private static final double kBackRightXPosInches = -11.125;
    private static final double kBackRightYPosInches = -11.125;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);

}
