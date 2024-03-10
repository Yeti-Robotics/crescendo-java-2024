package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class TrapReadyCommand extends Command {

    private final LEDSubsystem ledSubsystem;
    private final ClimberSubsystem climberSubsystem;

    public TrapReadyCommand(LEDSubsystem ledSubsystem, ClimberSubsystem climberSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.climberSubsystem = climberSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
//        if (climberSubsystem.atsetPoint) {
//            ledSubsystem.setRGB(0,205,50, 205);
//        }
    }

    @Override
    public boolean isFinished() {return true;}

    @Override
    public boolean runsWhenDisabled() {return true;}

    @Override
    public void end(boolean interrupted) {
    }
}
