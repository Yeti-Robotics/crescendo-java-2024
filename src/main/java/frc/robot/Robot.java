// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.commands.led.SetLEDToRGBCommand;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.AutoConstants;

import java.util.Arrays;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private static RobotContainer robotContainer;

    private AutoNamedCommands namedCommands;
    private ElevatorSubsystem elevatorSubsystem;
    private SetLEDToRGBCommand blueLedCommand;

    boolean buildAuto = true;

    AutoBuilder autoBuilder;
    private AutoConstants.AutoModes previousSelectedAuto;

    LimelightHelpers.Results lastResult;
    private static SendableChooser<AutoConstants.AutoModes> autoChooser;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
//        SignalLogger.enableAutoLogging(true);

        namedCommands = new AutoNamedCommands(robotContainer.intakeSubsystem, robotContainer.shooterSubsystem, robotContainer.pivotSubsystem, robotContainer.armSubsystem);
        namedCommands.registerCommands();
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(AutoConstants.AutoModes.BUMP_ONLY.name, AutoConstants.AutoModes.BUMP_ONLY);
        autoChooser.addOption(AutoConstants.AutoModes.BUMP_ONLY.name, AutoConstants.AutoModes.BUMP_ONLY);
//        autoChooser.addOption(AutoConstants.AutoModes.AMP_4_THREE_PIECE.name, AutoConstants.AutoModes.AMP_4_THREE_PIECE);
//        autoChooser.addOption(AutoConstants.AutoModes.MID_3_THREE_PIECE.name, AutoConstants.AutoModes.MID_3_THREE_PIECE);
//        autoChooser.addOption(AutoConstants.AutoModes.MID_3_TWO_PIECE.name, AutoConstants.AutoModes.MID_3_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_TWO_PIECE.name, AutoConstants.AutoModes.MID_SUB_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_THREE_PIECE.name, AutoConstants.AutoModes.MID_SUB_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_FOUR_PIECE.name, AutoConstants.AutoModes.MID_SUB_FOUR_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_2_PIECE.name, AutoConstants.AutoModes.SOURCE_SIDE_2_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_3_PIECE.name, AutoConstants.AutoModes.SOURCE_SIDE_3_PIECE);
//        autoChooser.addOption(AutoConstants.AutoModes.AMP_4_TWO_PIECE.name, AutoConstants.AutoModes.AMP_4_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_SHUTTLE_AMP.name, AutoConstants.AutoModes.SOURCE_SIDE_SHUTTLE_AMP);
        autoChooser.addOption(AutoConstants.AutoModes.SHUTTLE_3_SOURCE.name, AutoConstants.AutoModes.SHUTTLE_3_SOURCE);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        previousSelectedAuto = autoChooser.getSelected();
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name).withTimeout(15);
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);

//        DataLogManager.start();
//        DriverStation.startDataLog(DataLogManager.getLog());
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
        }

        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);
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

    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
//        resetCommandsAndButons();
        autonomousCommand.schedule();
        buildAuto = false;
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
            if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
                if (lastResult != null && lastResult.valid) {
                    Pose2d llPose = lastResult.getBotPose2d_wpiRed();
                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
            } else {
                if (lastResult != null && lastResult.valid) {
                    Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
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
