package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterCommand extends StartEndCommand {

    public ToggleShooterCommand(ShooterSubsystem shooterSubsystem) {
        super(() -> shooterSubsystem.shootFlywheel(0.6), shooterSubsystem::stopFlywheel);
        //        super(() -> shooterSubsystem.setMode(ShooterSubsystem.ShooterModes.AMP),
        // shooterSubsystem::stopFlywheel);
        addRequirements(shooterSubsystem);
    }
}
