// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Robot extends TimedRobot {
    private static RobotContainer robotContainer;
    private static SendableChooser<AutoConstants.AutoMode> autoChooser;
    LimelightHelpers.Results lastResult;
    private Command autonomousCommand;
    private AutoNamedCommands namedCommands;
    private AutoConstants.AutoMode previousSelectedAuto;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        namedCommands = new AutoNamedCommands(robotContainer.intakeSubsystem, robotContainer.shooterSubsystem, robotContainer.pivotSubsystem, robotContainer.armSubsystem);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }
        // note: setDefaultOption overwrites the name in the map, so we won't have duplicate options
        autoChooser.setDefaultOption(AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        previousSelectedAuto = autoChooser.getSelected();
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name).withTimeout(15);
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);


        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SmartDashboard.putNumber("shooterstate-position", 0.5);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
    }

    @Override
    public void disabledInit() {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
    }

    @Override
    public void disabledPeriodic() {
        if (previousSelectedAuto != autoChooser.getSelected()) {
            previousSelectedAuto = autoChooser.getSelected();
            autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);
        }

        System.out.println(previousSelectedAuto.name);
    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.getAlliance().isPresent()) {
            if (lastResult != null && lastResult.valid && Math.abs(robotContainer.drivetrain.getPigeon2().getRate()) < 230) {
                Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
                robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            }
        }
    }


    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
