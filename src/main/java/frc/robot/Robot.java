// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.led.SetLEDToRGBCommand;

import java.util.Set;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer robotContainer;
  private RobotContainer m_robotContainer;
  private SetLEDToRGBCommand blueLedCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
//    new SetLEDToRGBCommand(robotContainer.ledSubsystem, 128, 0, 128, 0.75, 0).schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {  }

  @Override
  public void disabledPeriodic() {

    System.out.println(robotContainer.climberSubsystem.getServo());

//    System.out.println(robotContainer.intakeSubsystem.getBeamBreak());
//    System.out.println(robotContainer.armSubsystem.getEnc());
    System.out.println(robotContainer.pivotSubsystem.getEncAngle());



  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
