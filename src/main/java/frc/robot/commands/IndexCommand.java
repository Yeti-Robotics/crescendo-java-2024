package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IndexCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public IndexCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        shooterSubsystem.spinNeo();
        intakeSubsystem.roll(-.4);


    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        intakeSubsystem.stop();
        shooterSubsystem.stopNeo();

    }
}
