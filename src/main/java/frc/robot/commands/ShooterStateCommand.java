package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;


public class ShooterStateCommand extends Command {
    private static final double pivotErrorMargin = 0.01;
    private static final double shooterErrorMargin = 10;
    private static final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));

    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final PivotSubsystem pivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final IntakeSubsystem intakeSubsystem;
    private double rps = 0;
    private double angle = 0;

    public ShooterStateCommand(CommandSwerveDrivetrain commandSwerveDrivetrain,
                               PivotSubsystem pivotSubsystem,
                               ShooterSubsystem shooterSubsystem,
                               IntakeSubsystem intakeSubsystem) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.pivotSubsystem = pivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsystem, this.shooterSubsystem, this.intakeSubsystem);
    }


    public void execute() {
        Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
        Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
        double distance = relativeSpeaker.getTranslation().getNorm();
        rps = ShooterConstants.SHOOTER_MAP().get(distance).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(distance).angle;

        SmartDashboard.putNumber("calc shooter angle", angle);
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angle", angle);

        shooterSubsystem.setVelocity(rps);
        pivotSubsystem.adjustPivotPositionTo(angle);
        intakeSubsystem.rollOut(-0.2);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivotSubsystem.getPivotPosition() - angle) < pivotErrorMargin &&
                Math.abs(shooterSubsystem.getVelocity() - rps) < shooterErrorMargin;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        intakeSubsystem.stop();
        pivotSubsystem.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF);
    }
}
