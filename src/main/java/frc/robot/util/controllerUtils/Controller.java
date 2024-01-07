package frc.robot.util.controllerUtils;

import edu.wpi.first.wpilibj.GenericHID;

public abstract class Controller extends GenericHID {
    public Controller(int port) {
        super(port);
    }

    public abstract double getLeftY();

    public abstract double getLeftX();

    public abstract double getRightY();

    public abstract double getRightX();
}