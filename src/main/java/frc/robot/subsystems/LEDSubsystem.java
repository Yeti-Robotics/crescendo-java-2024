package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final LEDStripStatus stripStatus;
    private final ShooterSubsystem shooterSubsystem;
    // I dont care if this is "bad practice" 😎
    private double brightness = LEDConstants.BRIGHTNESS;

    public LEDSubsystem(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        ledStrip = new AddressableLED(LEDConstants.ADDRESSABLE_LED);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        stripStatus = LEDStripStatus.ON;
        SmartDashboard.putNumber("Brightness", brightness);


        // Light states
        super.setDefaultCommand(
                new RunCommand(() -> {
                    if (shooterSubsystem.shootingState == ShooterSubsystem.ShootingState.NOT_SHOOTING) { // Add && if theta controller is at set point
                        setRGB(
                                0,
                                LEDConstants.READY_GREEN[1],
                                LEDConstants.READY_GREEN[2],
                                LEDConstants.READY_GREEN[3]
                        );
                        sendData();
                    } 
                    /*
                    else if (note_loaded) {
                        if note_loaded then display noteOrange color on LED's
                    }
                    */
                    else {
                        double red = LEDConstants.YETI_BLUE[1];
                        double green = LEDConstants.YETI_BLUE[2];
                        double blue = LEDConstants.YETI_BLUE[3];
                        ledBuffer.setRGB(0, (int) (red * brightness), (int) (green * brightness), (int) (blue * brightness));
                    }
                }, this));
    }


    @Override
    public void periodic() {
        // Apply brightness to the periodic update
        setBrightness(SmartDashboard.getNumber("Brightness", brightness));
        // For demonstration, setting default color
        setRGB(0, 255, 0, 0); // Red
        sendData();
    }

    public void setHSV(int i, int hue, int saturation, int value) {
        ledBuffer.setHSV(i, hue, saturation, (int) (value * brightness));
    }

    public void setRGB(int i, int red, int green, int blue) {
        ledBuffer.setRGB(i, (int) (red * brightness), (int) (green * brightness), (int) (blue * brightness));
    }

    public int getBufferLength() {
        return ledBuffer.getLength();
    }

    public void sendData() {
        ledStrip.setData(ledBuffer);
    }

    public void setBrightness(double brightness) {
        this.brightness = Math.max(0.0, Math.min(1.0, brightness)); // Ensure brightness is between 0 and 1
    }

    public enum LEDStripStatus {
        OFF,
        ON
    }
}
