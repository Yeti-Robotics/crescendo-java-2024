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
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.AutoConstants;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private static RobotContainer robotContainer;

    private AutoNamedCommands namedCommands;
    private AutoConstants.AutoMode previousSelectedAuto;

    LimelightHelpers.Results lastResult;
    private static SendableChooser<AutoConstants.AutoMode> autoChooser;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
//        SignalLogger.enableAutoLogging(true);
        System.out.println("ROBOT INIT START");

        namedCommands = new AutoNamedCommands(robotContainer.intakeSubsystem, robotContainer.shooterSubsystem, robotContainer.pivotSubsystem, robotContainer.armSubsystem);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }

        autoChooser.setDefaultOption(AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        previousSelectedAuto = autoChooser.getSelected();
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name).withTimeout(15);
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);


        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        System.out.println("ROBOT INIT END");
        SmartDashboard.putNumber("shooterstate-position", 0.5);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
       lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
//       DriverStation.refreshData();

    }

    @Override
    public void disabledInit() {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
//        resetCommandsAndButons();

//        Command disabledInitAuto = new PathPlannerAuto("bumpOnly").ignoringDisable(true);
//        disabledInitAuto.schedule();
    }

    @Override
    public void disabledPeriodic() {
        if (previousSelectedAuto != autoChooser.getSelected()) {
            previousSelectedAuto = autoChooser.getSelected();
            autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);
        }

//        System.out.println(robotContainer.shooterSubsystem.getBeamBreak());
//        System.out.println(previousSelectedAuto.name);
//        System.out.println(robotContainer.intakeSubsystem.getBeamBreak());


//        var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
//        if (DriverStation.getAlliance().isPresent()) {
//            if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
//                Pose2d llPose = lastResult.getBotPose2d_wpiRed();
//                if (lastResult.valid) {
//                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
//                }
//            } else {
//                Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
//                if (lastResult.valid) {
//                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
//                }
//            }
//        }

        System.out.println(previousSelectedAuto.name);
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
//        resetCommandsAndButons();
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
//        resetCommandsAndButons();
        autonomousCommand.cancel();
    }

    @Override
    public void teleopInit() {
        robotContainer.eventLoop.poll();
//        resetCommandsAndButons();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

//        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);

//        SignalLogger.start();
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
