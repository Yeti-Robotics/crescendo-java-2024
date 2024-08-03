package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;

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
     * @return {@code Command} instance
     */
    public Command setShooterState() {
        RobotDataPublisher<ShooterStateData> shooterStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
            Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
            double distance = relativeSpeaker.getTranslation().getNorm();
            return ShooterConstants.SHOOTER_MAP().get(distance);
        });

        return pivot.updatePivotPositionWith(shooterStatePublisher)
                .alongWith(shooter.updateVelocityWith(shooterStatePublisher));
    }

    @Deprecated
    public Command handoff() {
        return pivot.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF).andThen(
                new StartEndCommand(() -> arm.moveUp(.5), arm::stop).until(() ->
                        arm.getEnc() <= ArmConstants.ARM_HANDOFF_POSITION).andThen(
                        shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-0.35))
                ).until(shooter::getBeamBreak).andThen(intake.rollOut(1).withTimeout(0.2))
        );
    }
}
