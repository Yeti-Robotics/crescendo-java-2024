package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNamedCommands {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ArmSubsystem armSubsystem;


    public AutoNamedCommands() {
        registerCommands();
    }

    public void registerCommands() {
    NamedCommands.registerCommand("rollIn", new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop));
    NamedCommands.registerCommand("rollOut", new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop));
    NamedCommands.registerCommand("armUp", new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.75).andThen(armSubsystem::setMotorsBrake));
    NamedCommands.registerCommand("armDown", new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() <= 0.40).andThen(armSubsystem::setMotorsBrake));
    NamedCommands.registerCommand("shootOut", new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));

    }

    public static Command getAutoCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

}