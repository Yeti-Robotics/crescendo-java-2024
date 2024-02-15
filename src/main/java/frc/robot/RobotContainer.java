// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ControllerContainer controllerContainer = new ControllerContainer();
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());

    public RobotContainer() {
        configureBindings();
    }


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