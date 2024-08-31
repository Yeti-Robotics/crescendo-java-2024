package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.TurnToPoint;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotCommands {

    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final ArmSubsystem arm;

    public RobotCommands(IntakeSubsystem intake, PivotSubsystem pivot, ShooterSubsystem shooter, CommandSwerveDrivetrain commandSwerveDrivetrain, ArmSubsystem arm) {
        this.intake = intake;
        this.pivot = pivot;
        this.shooter = shooter;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.arm = arm;
    }


    private ShooterStateData getShooterData(Pose2d robotPose) {
        boolean isSpeaker = commandSwerveDrivetrain.getLatestPose().getX() < Constants.FieldConstants.Shuttle.shuttleLimit;

        Pose2d targetPose = AllianceFlipUtil.apply(
                isSpeaker ? Constants.FieldConstants.Speaker.speakerPose : Constants.FieldConstants.Shuttle.shuttleTarget
        );

        double relativeDistance = robotPose.relativeTo(targetPose).getTranslation().getNorm();

        InterpolatingTreeMap<Double, ShooterStateData> map = isSpeaker ? ShooterSubsystem.ShooterConstants.SHOOTER_MAP() : ShooterSubsystem.ShooterConstants.SHUTTLE_MAP();

        return map.get(relativeDistance);
    }

    private Translation2d getDriveTarget(Pose2d robotPose) {
        return AllianceFlipUtil.apply(
                robotPose.getX() < Constants.FieldConstants.Shuttle.shuttleLimit ?
                        Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d() : Constants.FieldConstants.Shuttle.shuttleTargetZone
        );
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     *
     * @return {@code Command} instance
     */
    public Command aimAndShoot(DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
        TurnToPoint poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        Supplier<ShooterStateData> shooterStateDataSupplier = () -> getShooterData(commandSwerveDrivetrain.getLatestPose());

        return commandSwerveDrivetrain.applyRequest(() -> {
                    poseAimRequest.setPointToFace(getDriveTarget(commandSwerveDrivetrain.getLatestPose()));
                    poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5);
                    return poseAimRequest;
                }).repeatedly()
                .alongWith(pivot.updatePivotPositionWith(shooterStateDataSupplier).repeatedly())
                .alongWith(shooter.updateVelocityWith(shooterStateDataSupplier).repeatedly());
    }


    public Command aim(DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
        TurnToPoint poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        return commandSwerveDrivetrain.runEnd(() -> {
            poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5);
            poseAimRequest.setPointToFace(getDriveTarget(commandSwerveDrivetrain.getLatestPose()));
            commandSwerveDrivetrain.setControl(poseAimRequest);
        }, () -> {
        });
    }

    public Command shoot() {
        RobotDataPublisher<ShooterStateData> shooterRobotPublisher = commandSwerveDrivetrain.observablePose()
                .map(pose2d -> {
                    boolean isSpeaker = commandSwerveDrivetrain.getLatestPose().getX() < Constants.FieldConstants.Shuttle.shuttleLimit;
                    Pose2d targetPose = AllianceFlipUtil.apply(
                            isSpeaker ?
                                    Constants.FieldConstants.Speaker.speakerPose : Constants.FieldConstants.Shuttle.shuttleTarget
                    );
                    Pose2d relativePose = pose2d.relativeTo(targetPose);

                    ShooterStateData data = isSpeaker ? ShooterSubsystem.ShooterConstants.SHOOTER_MAP().get(relativePose.getTranslation().getNorm()) :
                            ShooterSubsystem.ShooterConstants.SHUTTLE_MAP().get(relativePose.getTranslation().getNorm());

                    return data;
                });

        return pivot.updatePivotPositionWith(shooterRobotPublisher).alongWith(shooter.updateVelocityWith(shooterRobotPublisher));
    }

    @Deprecated
    public Command locationBasedAim(DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
        TurnToPoint poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        AtomicReference<Double> initTag = new AtomicReference<>((double) 0);

        return new FunctionalCommand(
                () -> {
                    initTag.set(LimelightHelpers.getFiducialID(VisionSubsystem.VisionConstants.LIMELIGHT_NAME));

                    Translation2d aimTarget = AllianceFlipUtil.apply(
                            commandSwerveDrivetrain.getLatestPose().getX() < Constants.FieldConstants.Shuttle.shuttleLimit ?
                                    Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d() : Constants.FieldConstants.Shuttle.shuttleTargetZone
                    );

                    poseAimRequest.setPointToFace(aimTarget);
                },
                () -> {
                    double currentTag = LimelightHelpers.getFiducialID(VisionSubsystem.VisionConstants.LIMELIGHT_NAME);
                    if (currentTag == initTag.get()) {
                        poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5);
                        commandSwerveDrivetrain.setControl(poseAimRequest);
                    }
                }, (bool) -> {
        }, () -> true, commandSwerveDrivetrain
        );
    }

    public Command locationBasedShooter() {
        RobotDataPublisher<ShooterStateData> shooterPublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
            final Pose2d shuttleTarget = AllianceFlipUtil.apply(new Pose2d(2.5, 7.0, Rotation2d.fromDegrees(0)));
            Pose2d relativeTarget = AllianceFlipUtil.apply(
                    commandSwerveDrivetrain.getLatestPose().getX() < Constants.FieldConstants.Shuttle.shuttleLimit ?
                            robotPose.relativeTo(Constants.FieldConstants.Speaker.speakerPose) : robotPose.relativeTo(Constants.FieldConstants.Shuttle.shuttleTarget)
            );
            double targetDistance = relativeTarget.getTranslation().getNorm();
            if (relativeTarget == AllianceFlipUtil.apply(robotPose.relativeTo(Constants.FieldConstants.Speaker.speakerPose))) {
                return ShooterSubsystem.ShooterConstants.SHOOTER_MAP().get(targetDistance);
            } else {
                return ShooterSubsystem.ShooterConstants.SHUTTLE_MAP().get(targetDistance);
            }
        });

        return pivot.updatePivotPositionWith(shooterPublisher).alongWith(shooter.updateVelocityWith(shooterPublisher));
    }

    public Command setShooterState() {
        RobotDataPublisher<ShooterStateData> shooterStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            Pose2d relativeSpeaker = robotPose.relativeTo(Constants.FieldConstants.Speaker.speakerPose);
            double distance = relativeSpeaker.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHOOTER_MAP().get(distance);
        });

        return pivot.updatePivotPositionWith(shooterStatePublisher)
                .alongWith(shooter.updateVelocityWith(shooterStatePublisher));
    }

    public Command setShuttleState() {
        RobotDataPublisher<ShooterStateData> shuttleStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            Pose2d relativeShuttleTarget = robotPose.relativeTo(Constants.FieldConstants.Shuttle.shuttleTarget);
            double distance = relativeShuttleTarget.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHUTTLE_MAP().get(distance);
        });
        return pivot.updatePivotPositionWith(shuttleStatePublisher).alongWith(shooter.updateVelocityWith(shuttleStatePublisher));
    }

    public Command handoff() {
        return pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF).andThen(
                arm.moveUpAndStop(.5).until(() ->
                        arm.getEnc() >= ArmSubsystem.ArmConstants.ARM_HANDOFF_POSITION).andThen(
                        shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-0.15))
                ).until(shooter::getBeamBreak)
        );
    }
}
