//Borrowed from Mechanical Advantage, FRC Team 6328
package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
    public static final double speakerPose = fieldWidth - Units.inchesToMeters(104.0);
    public static final double robotHeight = Units.feetToMeters(2.333);
    public static final double speakerHeightRelativeToBot = Units.inchesToMeters(77.9375) - robotHeight;

    public static final Translation2d ampCenter =
            new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

    /**
     * Staging locations for each note
     */
    public static final class StagingLocations {
        public static final double centerlineX = Units.inchesToMeters(fieldLength / 2);

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
        public static final Pose2d centerSpeakerOpening =
                new Pose2d(fieldWidth - Units.inchesToMeters(104.0), Units.inchesToMeters(9), new Rotation2d());

        // corners (blue alliance origin)
        public static final Translation3d topRightSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(238.815),
                        Units.inchesToMeters(13.091));

        public static final Translation3d topLeftSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(197.765),
                        Units.inchesToMeters(83.091));

        public static final Translation3d bottomRightSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
        public static final Translation3d bottomLeftSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

        public static final double aprilTagWidth = Units.inchesToMeters(6.50);
        public final static AprilTagFieldLayout aprilTags;

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
    public static Pose2d ampTag = new Pose2d(FieldConstants.ampCenter, Rotation2d.fromDegrees(0));

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
            ampTag = new Pose2d(FieldConstants.ampCenter, Rotation2d.fromDegrees(0));
            allianceAprilTags.addAll(aprilTagPoses.subList(1,6));
            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(7,12));
        } else {
            ampTag = new Pose2d(FieldConstants.ampCenter.getX(), 0, Rotation2d.fromDegrees(0));
            allianceAprilTags.addAll(aprilTagPoses.subList(7,12));
            opposingAllianceAprilTags.addAll(aprilTagPoses.subList(1,6));

        }
    }
}