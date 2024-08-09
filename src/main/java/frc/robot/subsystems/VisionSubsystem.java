package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    public double getX() {
        return LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_NAME);
    }

    public double getY() {
        return LimelightHelpers.getTY(Constants.VisionConstants.LIMELIGHT_NAME);
    }

    public LimelightHelpers.Results getTargetingResults() {
        return LimelightHelpers.getLatestResults(Constants.VisionConstants.LIMELIGHT_NAME).targetingResults;
    }

    public Command ledOff() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.LIMELIGHT_NAME));
    }

    public Command ledOn() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOn(Constants.VisionConstants.LIMELIGHT_NAME));
    }

    public Command blinkLimelight() {
        SequentialCommandGroup blinkCommands = new SequentialCommandGroup();

        for (int i = 0; i < Constants.VisionConstants.NUM_BLINKS; i++) {
            blinkCommands.addCommands(Commands.sequence(
                    ledOn(),
                    Commands.waitSeconds(Constants.VisionConstants.BLINK_DELAY_SECONDS),
                    ledOff(),
                    Commands.waitSeconds(Constants.VisionConstants.BLINK_DELAY_SECONDS)
                    ));
        }

        return blinkCommands;
    }
}