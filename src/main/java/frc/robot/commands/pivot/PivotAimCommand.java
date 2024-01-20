package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.constants.PivotConstants.*;
import static frc.robot.constants.FieldConstants.*;

public class PivotAimCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private double calcAngle;
    private double relativePoseY;
    private double relativePoseX;
    private double robotPoseY;
    private double robotPoseX;
    public PivotAimCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void initialize() {
        calcAngle = 0; //placeholder, set to bump value
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = speakerPose - visionSubsystem.getPose2d().getX();
        robotPoseX = visionSubsystem.getPose2d().getX();
        robotPoseY = visionSubsystem.getPose2d().getY();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
