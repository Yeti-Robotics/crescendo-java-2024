package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;

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

    /**
     * Requires
     * Sets shooter state in preparation for shooting
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
