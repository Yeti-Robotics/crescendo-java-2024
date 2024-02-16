// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

    import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.util.Units;

import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class DriveConstants {

    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_AZIMUTH = 1;
    public static final int FRONT_LEFT_ENCODER = 1;
    public static final boolean FRONT_LEFT_DRIVE_REVERSED = true;
    public static final double FRONT_LEFT_ENCODER_OFFSET = -18.457;
    public static final boolean FRONT_LEFT_ENCODER_REVERSED = false;

    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int FRONT_RIGHT_AZIMUTH = 3;
    public static final int FRONT_RIGHT_ENCODER = 2;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSED = false;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 127.881;
    public static final boolean FRONT_RIGHT_ENCODER_REVERSED = false;

    public static final int BACK_LEFT_DRIVE = 8;
    public static final int BACK_LEFT_AZIMUTH = 7;
    public static final int BACK_LEFT_ENCODER = 4;
    public static final boolean BACK_LEFT_DRIVE_REVERSED = true;
    public static final double BACK_LEFT_ENCODER_OFFSET = -350.596;
    public static final boolean BACK_LEFT_ENCODER_REVERSED = false;

    public static final int BACK_RIGHT_DRIVE = 6;
    public static final int BACK_RIGHT_AZIMUTH = 5;
    public static final int BACK_RIGHT_ENCODER = 3;
    public static final boolean BACK_RIGHT_DRIVE_REVERSED = false;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 131.836;
    public static final boolean BACK_RIGHT_ENCODER_REVERSED = false;

    public static final String FRONT_LEFT_MODULE_NAME = "front left";
    public static final String FRONT_RIGHT_MODULE_NAME = "front right";
    public static final String BACK_LEFT_MODULE_NAME = "back left";
    public static final String BACK_RIGHT_MODULE_NAME = "back right";

    public static final int GYRO = 1; //placeholder value

    public static final double DRIVE_MOTOR_P = 2.0; //placeholder from borealis
    public static final double DRIVE_MOTOR_I = 0.0; //placeholder from borealis
    public static final double DRIVE_MOTOR_D = 0.0; //placeholder from borealis
    public static final double DRIVE_MOTOR_KS = 0.743; //placeholder from borealis
    public static final double DRIVE_MOTOR_KV = 2.178; //placeholder from borealis
    public static final double DRIVE_MOTOR_KA = 0.406; //placeholder from borealis

    public static final double AZIMUTH_MOTOR_P = 5.0; //placeholder from borealis 3
    public static final double AZIMUTH_MOTOR_I = 0.0; //placeholder from borealis
    public static final double AZIMUTH_MOTOR_D = 0.00; //placeholder from borealis 0.01
    public static final double AZIMUTH_MOTOR_KS = 0.50; //placeholder from borealis 0.75
    public static final double AZIMUTH_MOTOR_KV = 0.35; //placeholder from borealis 0.7
    public static final double AZIMUTH_MOTOR_KA = 0.0; //placeholder from borealis

    public static final double DEGREES_TO_FALCON = 20.64 * 2048 / 360.0;
    public static final double SWERVE_X_REDUCTION = 1.0 / 6.75;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //0.1016

    public static final double MaFxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SWERVE_X_REDUCTION * WHEEL_DIAMETER * Math.PI; //placeholder

    public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
            withSupplyCurrentThreshold(55).withSupplyCurrentLimit(65).withSupplyTimeThreshold(0.1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);

    public static final double FRAME_PERIMETER = 27.0;
    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2); //PLACEHOLDER

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Front right
                    new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Back left
                    new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Back right
                    new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
            );
}