package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDToRGBCommand extends Command {

    private final LEDSubsystem ledSubsystem;
    private final int red, green, blue;

    public SetLEDToRGBCommand(LEDSubsystem ledSubsystem, int red, int green, int blue) {
        this.ledSubsystem = ledSubsystem;
        this.red = red;
        this.green = green;
        this.blue = blue;
        addRequirements();
    }

    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            ledSubsystem.setRGB(i, red, green, blue);
        }
        ledSubsystem.sendData();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {return true;}

    @Override
    public boolean runsWhenDisabled() {return true;}

    @Override
    public void end(boolean interrupted) {}
}