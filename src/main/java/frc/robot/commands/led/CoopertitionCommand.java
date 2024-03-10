package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;


public class CoopertitionCommand extends Command {
    private final LEDSubsystem ledSubsystem;
    public CoopertitionCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(this.ledSubsystem);

    }

    @Override
    public void initialize () {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess;
            // Set the value
            ledSubsystem.setRGB(0,20,120, 255);
        }
        // Increase by to make the rainbow "move"
        ledSubsystem.sendData();
    }
    @Override
    public void execute() {
        ledSubsystem.setRGB(0, 255, 0, 255);
        ledSubsystem.setBrightness(1);
        ledSubsystem.sendData();
    }

    @Override
    public boolean isFinished() {return true;}

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}