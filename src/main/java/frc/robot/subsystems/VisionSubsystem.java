package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

// does this need to be a subsystem? we aren't using any commands
public class VisionSubsystem extends SubsystemBase {
    public double xFinal;
    public double yFinal;

    public static final String LIMELIGHT_NAME = "limelight";


    public boolean hasTargets() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public double getX() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    public double getY() {
        return LimelightHelpers.getTY(LIMELIGHT_NAME);
    }

    public double[] getPose() {
        return LimelightHelpers.getBotPose(LIMELIGHT_NAME);
    }

    public Pose3d getPose3d() {
        return LimelightHelpers.getBotPose3d(LIMELIGHT_NAME);
    }

    public Pose2d getPose2d() {
        return LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
    }


    public double getYaw() {
        return getPose()[5];
    }

    public double getPitch() {
        return getPose()[4];
    }

    public double getID() {
        return LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    }

    public void setPipeline(int num) {
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, num);
    }

    public LimelightHelpers.Results getTargetingResults() {
        return LimelightHelpers.getLatestResults(LIMELIGHT_NAME).targetingResults;
    }

    public void ledOff() {
        LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
    }

    public void ledOn() {
        LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME);
    }
}