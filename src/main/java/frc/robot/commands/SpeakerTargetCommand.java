package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.constants.LEDConstants;
import static frc.robot.constants.FieldConstants.*;


public class SpeakerTargetCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final LEDSubsystem ledSubsystem;
    private double calcAngle;
    private double relativePoseY;
    private double relativePoseX;
    private double robotPoseX;
    public SpeakerTargetCommand(VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ledSubsystem = ledSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void initialize() {
        calcAngle = 0;
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = speakerPose - visionSubsystem.getPose2d().getX();
        robotPoseX = visionSubsystem.getPose2d().getX();
    }
    @Override
    public void execute() {
//        if (shooterSubsystem.shootingState == ShooterSubsystem.ShootingState.NOT_SHOOTING) { // Add && if theta controller is at set point
//            ledSubsystem.setRGB(
//                    0,
//                    LEDConstants.READY_GREEN[1],
//                    LEDConstants.READY_GREEN[2],
//                    LEDConstants.READY_GREEN[3]
//            );
//            ledSubsystem.sendData();
//        }
        if (true /*thetaController at setpoint*/) {
            shooterSubsystem.shootingState = ShooterSubsystem.ShootingState.READY_TO_SHOOT;
        }
        if (robotPoseX >= speakerPose + 20.6785 && robotPoseX <= speakerPose - 20.6875) {
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
