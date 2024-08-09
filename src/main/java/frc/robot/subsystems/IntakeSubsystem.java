package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeKraken;
    private final DigitalInput beamBreak;

    public IntakeSubsystem() {
        intakeKraken = new TalonFX(IntakeConstants.INTAKE_KRAKEN_ID, "canivoreBus");
        var intakeConfigurator = intakeKraken.getConfigurator();
        var configs = new TalonFXConfiguration();

        beamBreak = new DigitalInput(2);
        configs.MotorOutput.Inverted = IntakeConstants.INTAKE_INVERSION;
        configs.MotorOutput.NeutralMode = IntakeConstants.INTAKE_NEUTRAL_MODE;
        configs.FutureProofConfigs = TalonFXConstants.TALON_FUTURE_PROOF;
        intakeKraken.getRotorVelocity().waitForUpdate(IntakeConstants.INTAKE_VELOCITY_STATUS_FRAME);
        intakeKraken.getRotorPosition().waitForUpdate(IntakeConstants.INTAKE_POSITION_STATUS_FRAME);
        intakeConfigurator.apply(configs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("intake kraken", intakeKraken);
        SmartDashboard.putData("intake beam break", beamBreak);
    }

    private void setIntakeSpeed(double speed) {
        intakeKraken.set(speed);
    }

    private void stop() {
        intakeKraken.stopMotor();
    }

    private Command roll(double vel) {
        return startEnd(() -> setIntakeSpeed(vel), this::stop);
    }

    /**
     * Sucks up note into the robot
     *
     * @param vel positive speed in RPS
     * @return {@code Command} instance
     */
    public Command rollIn(double vel) {
        // TODO: add some kind of warning logging if a negative value is passed

        return roll(Math.abs(vel));
    }

    /**
     * Sucks up note into the robot
     *
     * @param vel negative speed in RPS
     * @return {@code Command} instance
     */
    public Command rollOut(double vel) {
        // TODO: add some kind of warning logging if a positive value is passed

        return roll(-Math.abs(vel));
    }
}
