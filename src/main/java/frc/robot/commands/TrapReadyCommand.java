package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class TrapReadyCommand extends Command {

    private final LEDSubsystem ledSubsystem;

    public TrapReadyCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (ClimberSystem.atsetPoint) {
            ledSubsystem.setRGB(0,205,50, 205);
        }
    }

    @Override
    public boolean isFinished() {return true;}

    @Override
    public boolean runsWhenDisabled() {return true;}

    @Override
    public void end(boolean interrupted) {
    }
}
