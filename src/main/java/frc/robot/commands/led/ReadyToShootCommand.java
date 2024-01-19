package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;


public class ReadyToShootCommand extends Command {
    private final LEDSubsystem ledSubsystem;

    public ReadyToShootCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.ledSubsystem);
    }
    @Override
    public void initialize () {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess;
            // Set the value
            ledSubsystem.setRGB(0,255, 255, 0);
        }
        // Increase by to make the rainbow "move"
        ledSubsystem.sendData();
    }
    @Override

    public void execute() {
//        if ()
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                ledSubsystem.setRGB(0,50, 205, 50);
            }
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