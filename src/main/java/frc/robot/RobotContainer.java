// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.ToggleShooterCommand;
=======
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IndexCommand;
>>>>>>> 72204fa7c9963c9c868b8ce73260a55bc9a0052a
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

<<<<<<< HEAD
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ControllerContainer controllerContainer = new ControllerContainer();
  ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
  public RobotContainer() {
    configureBindings();
  }
=======
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
>>>>>>> 72204fa7c9963c9c868b8ce73260a55bc9a0052a

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ControllerContainer controllerContainer = new ControllerContainer();
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());

<<<<<<< HEAD
    buttonHelper.createButton(1, 0, new StartEndCommand(() -> intakeSubsystem.roll(.50), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
    buttonHelper.createButton(2, 0, new StartEndCommand(() -> intakeSubsystem.roll(-.5), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
    buttonHelper.createButton(6, 0, new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopNeo), MultiButton.RunCondition.WHILE_HELD);
    buttonHelper.createButton(4, 0, new StartEndCommand(() -> shooterSubsystem.motionMagicTest(75), shooterSubsystem::stopFlywheel), MultiButton.RunCondition.WHILE_HELD);
    buttonHelper.createButton(3, 0, new IndexCommand(shooterSubsystem, intakeSubsystem), MultiButton.RunCondition.WHILE_HELD);
=======
    public RobotContainer() {
        configureBindings();
    }
>>>>>>> 72204fa7c9963c9c868b8ce73260a55bc9a0052a


    private void configureBindings() {

        buttonHelper.createButton(1, 0, new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(2, 0, new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(6, 0, new StartEndCommand(() -> shooterSubsystem.motionMagicTest(25), shooterSubsystem::stopFlywheel), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(7, 0, new IndexCommand(shooterSubsystem, intakeSubsystem), MultiButton.RunCondition.WHILE_HELD);
    }

        public Command getAutonomousCommand () {
            return new InstantCommand();
        }

}