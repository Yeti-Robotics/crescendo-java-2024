package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public IndexCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Ensure the subsystems are not null before accessing their methods
        if (shooterSubsystem != null && intakeSubsystem != null) {
            shooterSubsystem.spinNeo();
            intakeSubsystem.roll(-0.4);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the subsystems are not null before accessing their methods
        if (intakeSubsystem != null) {
            intakeSubsystem.stop();
        }
        if (shooterSubsystem != null) {
            shooterSubsystem.stopNeo();
        }
    }
}
