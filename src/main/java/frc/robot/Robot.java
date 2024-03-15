// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.PixelFormat;
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
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.AutoConstants;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private AutoNamedCommands namedCommands;
    private ElevatorSubsystem elevatorSubsystem;
    private RobotContainer m_robotContainer;
    private SetLEDToRGBCommand blueLedCommand;

    AutoBuilder autoBuilder;
    private AutoConstants.AutoModes previousSelectedAuto;

    private static SendableChooser<AutoConstants.AutoModes> autoChooser;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        SignalLogger.enableAutoLogging(true);

        // VisionConstants.CAMERA_NAME in config
//    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
//    usbCamera.setResolution(854, 480);
//    usbCamera.setFPS(30);
//    CameraServer.startAutomaticCapture(usbCamera);
        UsbCamera driverCam = CameraServer.startAutomaticCapture();
        driverCam.setVideoMode(PixelFormat.kYUYV, 176, 144, 30);
        driverCam.setConnectVerbose(1);
//    new SetLEDToRGBCommand(robotContainer.ledSubsystem, 128, 0, 128, 0.75, 0).schedule();
        namedCommands = new AutoNamedCommands(robotContainer.intakeSubsystem, robotContainer.shooterSubsystem, robotContainer.pivotSubsystem, robotContainer.armSubsystem);
        namedCommands.registerCommands();
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(AutoConstants.AutoModes.MID_3_THREE_PIECE.name, AutoConstants.AutoModes.MID_3_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.AMP_4_THREE_PIECE.name, AutoConstants.AutoModes.AMP_4_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_3_THREE_PIECE.name, AutoConstants.AutoModes.MID_3_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_3_TWO_PIECE.name, AutoConstants.AutoModes.MID_3_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_TWO_PIECE.name, AutoConstants.AutoModes.MID_SUB_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_THREE_PIECE.name, AutoConstants.AutoModes.MID_SUB_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.MID_SUB_FOUR_PIECE.name, AutoConstants.AutoModes.MID_SUB_FOUR_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_2_PIECE.name, AutoConstants.AutoModes.SOURCE_SIDE_2_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_3_PIECE.name, AutoConstants.AutoModes.SOURCE_SIDE_3_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.AMP_4_TWO_PIECE.name, AutoConstants.AutoModes.AMP_4_TWO_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_SIDE_SHUTTLE_AMP.name, AutoConstants.AutoModes.SOURCE_SIDE_SHUTTLE_AMP);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        previousSelectedAuto = autoChooser.getSelected();
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        SignalLogger.stop();
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
    }

    @Override
    public void disabledPeriodic() {
        if (previousSelectedAuto != autoChooser.getSelected()) {
            previousSelectedAuto = autoChooser.getSelected();
        }
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);
        System.out.println(robotContainer.intakeSubsystem.getBeamBreak());
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
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);

        SignalLogger.start();

    }

    @Override
    public void teleopPeriodic() {

        var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
                Pose2d llPose = lastResult.getBotPose2d_wpiRed();
                if (lastResult.valid) {
                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
            } else {
                Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
                if (lastResult.valid) {
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