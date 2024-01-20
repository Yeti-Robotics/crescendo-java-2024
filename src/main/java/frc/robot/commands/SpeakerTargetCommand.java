package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.constants.FieldConstants.*;


public class SpeakerTargetCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private double calcAngle;
    private double relativePoseY;
    private double relativePoseX;
    public SpeakerTargetCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void initialize() {
        calcAngle = 0;
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = speakerPose - visionSubsystem.getPose2d().getX();
    }
    @Override
    public void execute() {
        visionSubsystem.getPose2d().getY();
        visionSubsystem.getPose2d().getX();
        if (visionSubsystem.getPose2d().getX() >= speakerPose + 20.6785 && visionSubsystem.getPose2d().getX() <= speakerPose - 20.6875) {
            calcAngle = 0.0;
        }
        else {
            calcAngle = Math.atan2(relativePoseX, relativePoseY);
        }

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
