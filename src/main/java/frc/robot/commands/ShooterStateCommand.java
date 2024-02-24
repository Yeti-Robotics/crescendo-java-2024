package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterStateCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final PivotSubsystem pivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private double poseY = 0;
    private double rps = 0;
    private double angle = 0;
    public ShooterStateCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.pivotSubsystem = pivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.commandSwerveDrivetrain, this.pivotSubsystem);
    }

    @Override
    public void initialize() {

        poseY = commandSwerveDrivetrain.getState().Pose.getY();
        rps = ShooterConstants.SHOOTER_MAP().get(poseY).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(poseY).angle;

    }

    @Override
    public void execute() {

        shooterSubsystem.setVelocity(rps);
        pivotSubsystem.setPosition(angle);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
    }
}
