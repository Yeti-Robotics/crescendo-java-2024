package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.AllianceFlipUtil;

public class ShuttleStateCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final PivotSubsystem pivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private double rps = 0;
    private double angle = 0;
    private final double botAngle = 0;
    private Pose2d shuttleTarget;

    public ShuttleStateCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.pivotSubsystem = pivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsystem, this.shooterSubsystem, this.visionSubsystem);
    }

    @Override
    public void initialize() {
        shuttleTarget = AllianceFlipUtil.apply(new Pose2d(2.5, 1.0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void execute() {
        Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
        Pose2d relativeShuttleTarget = robotPose.relativeTo(shuttleTarget);
        double distance = relativeShuttleTarget.getTranslation().getNorm();
        System.out.println(distance);
        rps = ShooterConstants.SHUTTLE_MAP().get(distance).rps;
        angle = ShooterConstants.SHUTTLE_MAP().get(distance).angle;
        shooterSubsystem.setVelocity(rps);
        pivotSubsystem.setPivotPosition(angle);
        intakeSubsystem.roll(-.2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
        intakeSubsystem.stop();
        pivotSubsystem.setPivotPosition(0.5);
    }
}
