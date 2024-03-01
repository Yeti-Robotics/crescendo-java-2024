// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.led.SetLEDToRGBCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.util.LimelightHelpers;

import java.util.List;
import java.util.Set;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private RobotContainer m_robotContainer;
  AutoBuilder autoBuilder;
  private SetLEDToRGBCommand blueLedCommand;
  private AutoConstants.AutoModes previousSelectedAuto;

  private static SendableChooser<AutoConstants.AutoModes> autoChooser;
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption(AutoConstants.AutoModes.TESTING.name, AutoConstants.AutoModes.TESTING);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    previousSelectedAuto = autoChooser.getSelected();


   autonomousCommand =  AutoBuilder.buildAuto(previousSelectedAuto.name);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

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
  public void disabledInit() {  }

  @Override
  public void disabledPeriodic() {


    System.out.println(robotContainer.drivetrain.getState().Pose.toString());
    System.out.println(robotContainer.climberSubsystem.getServo());




  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();


    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
