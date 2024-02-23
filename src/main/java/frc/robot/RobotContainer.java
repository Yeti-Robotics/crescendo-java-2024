// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.pivot.PivotAimCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;

public class RobotContainer {
  public final VisionSubsystem visionSubsystem;
  public final PivotSubsystem pivotSubsystem;
  public final LEDSubsystem ledSubsystem;
  ControllerContainer controllerContainer = new ControllerContainer();
  ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
  public RobotContainer() {
    visionSubsystem = new VisionSubsystem();
    pivotSubsystem = new PivotSubsystem();
    ledSubsystem = new LEDSubsystem();
    pivotSubsystem.setDefaultCommand(new PivotAimCommand(visionSubsystem,pivotSubsystem,ledSubsystem));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
