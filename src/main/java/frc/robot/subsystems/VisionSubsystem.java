package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    public double xFinal;
    public double yFinal;




public boolean hasTargets() {
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
}

public double getX() {
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
}

public double getY() {
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
}

public double[] getPose(){
    return LimelightHelpers.getBotPose(VisionConstants.LIMELIGHT_NAME);
}

public Pose3d getPose3d() {
    return LimelightHelpers.getBotPose3d(VisionConstants.LIMELIGHT_NAME);
}

public Pose2d getPose2d() {
    return LimelightHelpers.getBotPose2d(VisionConstants.LIMELIGHT_NAME);
}

public double getYaw() {
    return getPose()[5];
}

public double getPitch() {
    return getPose()[4];
}

    public Command ledOff() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME));
    }

    public Command ledOn() {
        return runOnce(() -> LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME));
    }

public double getID() {
    return LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME);
}

public void setPipeline(int num){
    LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME, num);
}

    public Command blinkLimelight() {
        SequentialCommandGroup blinkCommands = new SequentialCommandGroup();

        for (int i = 0; i < VisionConstants.NUM_BLINKS; i++) {
            blinkCommands.addCommands(Commands.sequence(
                    ledOn(),
                    Commands.waitSeconds(VisionConstants.BLINK_DELAY_SECONDS),
                    ledOff(),
                    Commands.waitSeconds(VisionConstants.BLINK_DELAY_SECONDS)
            ));
        }

        return blinkCommands;
    }

}