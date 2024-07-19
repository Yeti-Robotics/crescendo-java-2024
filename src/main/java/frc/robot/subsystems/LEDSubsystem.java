package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    public LEDStripStatus stripStatus;
    private double brightness = 1.0; // Default to full brightness

    public LEDSubsystem() {
        ledStrip = new AddressableLED(LEDConstants.ADDRESSABLE_LED);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        stripStatus = LEDStripStatus.ON;
        SmartDashboard.putNumber("Brightness", brightness);
    }

    @Override
    public void periodic() {
        // Apply brightness to the periodic update
        // setBrightness(SmartDashboard.getNumber("Brightness", brightness));
        // For demonstration, setting default color
        // setRGB(0, 255, 0, 0); // Red
        // sendData();
    }

    public void setHSV(int i, int hue, int saturation, int value) {
        ledBuffer.setHSV(i, hue, saturation, (int) (value * brightness));
    }

    public void setRGB(int i, int red, int green, int blue) {
        ledBuffer.setRGB(
                i, (int) (red * brightness), (int) (green * brightness), (int) (blue * brightness));
    }

    public int getBufferLength() {
        return ledBuffer.getLength();
    }

    public void sendData() {
        ledStrip.setData(ledBuffer);
    }

    public void setBrightness(double brightness) {
        this.brightness =
                Math.max(0.0, Math.min(1.0, brightness)); // Ensure brightness is between 0 and 1
    }

    public void flashLimeLight() {}

    public enum LEDStripStatus {
        OFF,

        ON
    }
}
