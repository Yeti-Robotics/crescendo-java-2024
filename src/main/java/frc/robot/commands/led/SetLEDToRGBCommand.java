package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDToRGBCommand extends Command {

    private final LEDSubsystem ledSubsystem;
    private final int red, green, blue;
    private final double brightness;
    private final long delayMillis;

    private long lastUpdateTime;

    public SetLEDToRGBCommand(
            LEDSubsystem ledSubsystem,
            int red,
            int green,
            int blue,
            double brightness,
            long delayMillis) {
        this.ledSubsystem = ledSubsystem;
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.brightness = brightness;
        this.delayMillis = delayMillis;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime >= delayMillis) {
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                // Adjust color with brightness
                int adjustedRed = (int) (red * brightness);
                int adjustedGreen = (int) (green * brightness);
                int adjustedBlue = (int) (blue * brightness);
                ledSubsystem.setRGB(i, adjustedRed, adjustedGreen, adjustedBlue);
            }
            ledSubsystem.sendData();
            lastUpdateTime = currentTime;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
