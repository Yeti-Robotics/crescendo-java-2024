package frc.robot.commands.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.constants.FieldConstants.Speaker.centerSpeakerOpening;
import static frc.robot.constants.PivotConstants.*;
import static frc.robot.constants.FieldConstants.*;

public class PivotAimCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final LEDSubsystem ledSubsystem;
    private double relativePoseY;
    private double relativePoseX;
    private double robotPoseY;
    private double robotPoseX;
    private double hypoGroundLength;
    public double vertAngle;

    private PivotConstants.PivotPositions pivotPositions = PivotConstants.PivotPositions.BUMPFIRE;
    private PivotPositions positions;
    public PivotAimCommand(VisionSubsystem visionSubsystem,PivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.ledSubsystem = ledSubsystem;
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        hypoGroundLength = 0;
        vertAngle = 0;
    }

    @Override
    public void execute() {
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = centerSpeakerOpening.getX() - visionSubsystem.getPose2d().getX();
        robotPoseX = visionSubsystem.getPose2d().getX();
        robotPoseY = visionSubsystem.getPose2d().getY();
        hypoGroundLength = Math.sqrt((relativePoseX*relativePoseX)+(relativePoseY*relativePoseY));
        vertAngle = Math.toDegrees((Math.atan2(Units.inchesToMeters(speakerHeightRelativeToBot), hypoGroundLength)));
        //pivotSubsystem.setPosition(pivotSubsystem.vertAngle);
        System.out.println(vertAngle);
    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
