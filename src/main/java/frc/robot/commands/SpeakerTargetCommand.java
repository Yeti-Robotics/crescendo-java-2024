package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.constants.LEDConstants;
import frc.robot.util.LimelightHelpers;

import java.sql.Driver;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.FieldConstants.Speaker.centerSpeakerOpening;


public class SpeakerTargetCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final LEDSubsystem ledSubsystem;
    private double calcAngle;
    public double relativePoseY;
    public double relativePoseX;
    public double robotPoseX;

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
    }
    @Override
    public void execute() {
        relativePoseY = fieldLength - visionSubsystem.getPose2d().getY();
        relativePoseX = centerSpeakerOpening.getX() - visionSubsystem.getPose2d().getX();
        robotPoseX = visionSubsystem.getPose2d().getX();
//        if (shooterSubsystem.shootingState == ShooterSubsystem.ShootingState.NOT_SHOOTING) { // Add && if theta controller is at set point
//            ledSubsystem.setRGB(
//                    0,
//                    LEDConstants.READY_GREEN[1],
//                    LEDConstants.READY_GREEN[2],
//                    LEDConstants.READY_GREEN[3]
//       );
//            LedSubsystem.sendData();
//        }
        if (true /*thetaController at setpoint*/) {
            shooterSubsystem.shootingState = ShooterSubsystem.ShootingState.READY_TO_SHOOT;
        }
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            if (robotPoseX <= (fieldWidth - centerSpeakerOpening.getX()) + 0.2 &&
                    robotPoseX >= (fieldWidth - centerSpeakerOpening.getX()) - 0.2) {
                calcAngle = 0.0;
            }
            else {
                calcAngle = Math.toDegrees(Math.atan2(relativePoseX, relativePoseY));
            }
            System.out.println("Blue Speaker Pose: " + (fieldWidth - centerSpeakerOpening.getX()));
        }
        else {
            if (robotPoseX <= centerSpeakerOpening.getX() + 0.2 && robotPoseX >= centerSpeakerOpening.getX() - 0.2) {
                calcAngle = 0.0;
            }
            else {
                calcAngle = Math.toDegrees(Math.atan2(relativePoseX, relativePoseY));
            }
            System.out.println("Red Speaker Pose: " + centerSpeakerOpening.getX());
        }


        System.out.println("Calc Angle: " + calcAngle);
        System.out.println("Robot Pose X: " + robotPoseX);

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
