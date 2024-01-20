package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsytemSubsystem;


public class BumpShooterPivotCommand extends Command {
    private final PivotSubsytemSubsystem pivotSubsytemSubsystem = PivotSubsytemSubsystem.getInstance();

    public BumpShooterPivotCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsytemSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
