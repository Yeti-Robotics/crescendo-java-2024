package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    public double getX() {
        return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
    }

    public double getY() {
        return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
    }

    public LimelightHelpers.Results getTargetingResults() {
        return LimelightHelpers.getLatestResults(VisionConstants.LIMELIGHT_NAME).targetingResults;
    }

    public Command ledOff() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME));
    }

    public Command ledOn() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME));
    }

    public Command blinkLimelight() {
        SequentialCommandGroup blinkCommands = new SequentialCommandGroup();
        blinkCommands.addRequirements(this);

        for (int i = 0; i < VisionConstants.NUM_BLINKS; i++) {
            blinkCommands.addCommands(Commands.sequence(
                    ledOn(),
                    Commands.waitSeconds(VisionConstants.BLINK_DELAY),
                    ledOff()
            ));
        }

        return blinkCommands;
    }
}