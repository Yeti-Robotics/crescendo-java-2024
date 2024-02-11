package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.util.GeometryUtils;

public class DrivetrainSubsystem extends SubsystemBase {

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private final SwerveModulePosition[] positions;
    private final SwerveDrivePoseEstimator odometer;
    private final Pigeon2 gyro;
    private ChassisSpeeds chassisSpeeds;

    public DrivetrainSubsystem() {
        frontLeftModule = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE,
                DriveConstants.FRONT_LEFT_DRIVE_REVERSED, DriveConstants.FRONT_LEFT_AZIMUTH,
                DriveConstants.FRONT_LEFT_ENCODER, SensorDirectionValue.CounterClockwise_Positive,
                DriveConstants.FRONT_LEFT_ENCODER_OFFSET);
        frontRightModule = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE,
                DriveConstants.FRONT_RIGHT_DRIVE_REVERSED, DriveConstants.FRONT_RIGHT_AZIMUTH,
                DriveConstants.FRONT_RIGHT_ENCODER, SensorDirectionValue.CounterClockwise_Positive,
                DriveConstants.FRONT_RIGHT_ENCODER_OFFSET);

        backLeftModule = new SwerveModule(DriveConstants.BACK_LEFT_DRIVE,
                DriveConstants.BACK_LEFT_DRIVE_REVERSED, DriveConstants.BACK_LEFT_AZIMUTH,
                DriveConstants.BACK_LEFT_ENCODER, SensorDirectionValue.CounterClockwise_Positive,
                DriveConstants.BACK_LEFT_ENCODER_OFFSET);

        backRightModule = new SwerveModule(DriveConstants.BACK_RIGHT_DRIVE,
                DriveConstants.BACK_RIGHT_DRIVE_REVERSED, DriveConstants.BACK_RIGHT_AZIMUTH,
                DriveConstants.BACK_RIGHT_ENCODER, SensorDirectionValue.CounterClockwise_Positive,
                DriveConstants.BACK_RIGHT_ENCODER_OFFSET);

        positions = new SwerveModulePosition[4];
        gyro = new Pigeon2(DriveConstants.GYRO);
        odometer = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, gyro.getRotation2d(), positions, new Pose2d(), VecBuilder.fill(0.3,0.3,0.3),
                VecBuilder.fill(0.4,0.4,0.4));

        chassisSpeeds = new ChassisSpeeds(0,0,0);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                //gyro zero
            } catch (Exception e){
                System.out.println("FAILED TO ZERO");
            }
        }).start();


        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometer,
                this::getChassisSpeeds,
                this::runVel,
                new HolonomicPathFollowerConfig(
                        AutoConstants.TRANSLATION_CONTROLLER,
                        AutoConstants.THETA_CONTROLLER,
                        AutoConstants.MAX_VELOCITY,
                        Units.inchesToMeters(22.25),
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );

    }


    public void zeroGyroscope() {
        gyro.reset();
    }

    public void runVel(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] desiredStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    public Rotation2d getGyroscopeHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getRoll().getValue());
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }

    public void resetOdometer(Pose2d pose) {
        updateSwerveModulePositions();
        odometer.resetPosition(getGyroscopeHeading(), positions, pose);
    }

    public void updateOdometerWithVision(Pose2d visionPose, double timestamp) {
        odometer.addVisionMeasurement(visionPose, timestamp);
    }

    public void drive(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) { //Jimmy's 5727 2nd order swerve kinematics implementation <3
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
                new Pose2d(
                        originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                        originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
                new ChassisSpeeds(
                        twistForPose.dx / LOOP_TIME_S,
                        twistForPose.dy / LOOP_TIME_S,
                        twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    public ChassisSpeeds getChassisSpeeds() {
        chassisSpeeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        );
        chassisSpeeds = correctForDynamics(chassisSpeeds);
        return chassisSpeeds;
    }



    public void stop() {
        drive(
                DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds())
        );
    }

    public static double modifyAxis(double value, int pow) {
        if (Math.abs(value) <= OIConstants.DEADBAND) {
            return 0.0;
        }

        return Math.copySign(Math.pow(value, pow), value);
    }

    public void updateSwerveModulePositions() {
        positions[0] = frontLeftModule.getPosition();
        positions[1] = frontRightModule.getPosition();
        positions[2] = backLeftModule.getPosition();
        positions[3] = backRightModule.getPosition();
    }

    @Override
    public void periodic() {
        updateSwerveModulePositions();
        odometer.update(getGyroscopeHeading(), positions);
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("pose", () -> String.format("Pose (x,y): (%.2f, %.2f) Rotation(deg): %.2f", getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()), null);


    }
}

