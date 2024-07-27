package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

public class RobotCommands {

    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    public RobotCommands(IntakeSubsystem intake, PivotSubsystem pivot, ElevatorSubsystem elevator, ShooterSubsystem shooter, VisionSubsystem vision, CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.intake = intake;
        this.pivot = pivot;
        this.elevator = elevator;
        this.shooter = shooter;
        this.vision = vision;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     * @return {@code Command} instance
     */
    public Command setShooterState() {
        Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
        final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
        Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
        double distance = relativeSpeaker.getTranslation().getNorm();
        double rps = ShooterConstants.SHOOTER_MAP().get(distance).rps;
        double angle = ShooterConstants.SHOOTER_MAP().get(distance).angle;

        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angle", angle);

        return Commands.parallel(
                intake.rollOut(-0.2),
                shooter.setVelocityAndStop(rps),
                pivot.adjustPivotPositionTo(angle)
        );
    }

    @Deprecated
    public Command handoff() {
        return pivot.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF)
                .andThen();
    }
}
