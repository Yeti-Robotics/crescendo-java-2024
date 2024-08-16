package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class RobotCommands {

    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final ArmSubsystem arm;

    public RobotCommands(IntakeSubsystem intake, PivotSubsystem pivot, ElevatorSubsystem elevator, ShooterSubsystem shooter, VisionSubsystem vision, CommandSwerveDrivetrain commandSwerveDrivetrain, ArmSubsystem arm) {
        this.intake = intake;
        this.pivot = pivot;
        this.elevator = elevator;
        this.shooter = shooter;
        this.vision = vision;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.arm = arm;
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     *
     * @return {@code Command} instance
     */
    public Command setShooterState() {
        RobotDataPublisher<ShooterStateData> shooterStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
            Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
            double distance = relativeSpeaker.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHOOTER_MAP().get(distance);
        });

        return pivot.updatePivotPositionWith(shooterStatePublisher)
                .alongWith(shooter.updateVelocityWith(shooterStatePublisher));
    }

    public Command locationAim(DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
        TurnToPoint poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        AtomicReference<Double> initTag = new AtomicReference<>((double) 0);

        return new FunctionalCommand(
                () -> {
                    initTag.set(LimelightHelpers.getFiducialID(VisionSubsystem.VisionConstants.LIMELIGHT_NAME));

                    Translation2d aimTarget = AllianceFlipUtil.apply(
                            commandSwerveDrivetrain.getLatestPose().getX() > 5 ?
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
                }, (bool) -> {}, () -> true,commandSwerveDrivetrain
                );
    }

    public Command setShuttleState() {
        RobotDataPublisher<ShooterStateData> shuttleStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d shuttleTarget = AllianceFlipUtil.apply(new Pose2d(2.5, 7.0, Rotation2d.fromDegrees(0)));
            Pose2d relativeShuttleTarget = robotPose.relativeTo(shuttleTarget);
            double distance = relativeShuttleTarget.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHUTTLE_MAP().get(distance);
        });
        return pivot.updatePivotPositionWith(shuttleStatePublisher).alongWith(shooter.updateVelocityWith(shuttleStatePublisher));
    }

    public Command handoff() {
        return pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF).andThen(
                new StartEndCommand(() -> arm.moveUp(.5), arm::stop).until(() ->
                        arm.getEnc() >= ArmSubsystem.ArmConstants.ARM_HANDOFF_POSITION).andThen(
                        shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-0.15))
                ).until(shooter::getBeamBreak)
        );
    }
}
