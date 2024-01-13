package frc.robot.subsystems;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase implements Sendable{

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private int r, g, b;
    public enum LEDStripStatus {
        OFF,
        ON
    }

    public LEDStripStatus stripStatus;

    public LEDSubsystem(){
        ledStrip = new AddressableLED(LEDConstants.ADDRESSABLE_LED);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        stripStatus = LEDStripStatus.ON;
        SmartDashboard.putNumber("r", r);
        SmartDashboard.putNumber("g", g);
        SmartDashboard.putNumber("b", b);
    }

    @Override
    public void periodic() {}
    
    public void setHSV (int i, int hue, int saturation, int value) {
        ledBuffer.setHSV(i, hue, saturation, value);
    }

    public void setRGB(int i, int red, int green, int blue) {ledBuffer.setRGB(i, red, green, blue);
    }

    public int getBufferLength () {return ledBuffer.getLength();
    }

    public void sendData () {ledStrip.setData(ledBuffer);
    }
}

