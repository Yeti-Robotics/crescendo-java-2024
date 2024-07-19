package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class BlinkLimeLightCommand extends SequentialCommandGroup {

    public BlinkLimeLightCommand() {
        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME)),
                        new WaitCommand(0.25),
                        new InstantCommand(
                                () -> LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME)),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> System.out.println("Blink Command First blink")),
                        new InstantCommand(
                                () -> LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME)),
                        new WaitCommand(0.25),
                        new InstantCommand(
                                () -> LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME))),
                new InstantCommand(() -> System.out.println("Blink Command Finished")));
    }
}
