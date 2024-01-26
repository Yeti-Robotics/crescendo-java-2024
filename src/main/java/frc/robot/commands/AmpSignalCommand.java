package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class AmpSignalCommand extends Command {

   private final LEDSubsystem ledSubsystem;

    public AmpSignalCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

//    @Override
////    public void execute() {
//        if (AmpReady) {
//            ledSubsystem.setRGB(0, 255, 0, 255);
//        }
//    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
