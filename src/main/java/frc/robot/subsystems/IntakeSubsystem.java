package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX intakeKraken;

    DigitalInput beamBreak;
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
        if (getBeamBreak()) {
            LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.LIMELIGHT_NAME);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
        }
    }

    public void rollIn(double speed) {
        intakeKraken.set(Math.abs(speed));
    }

    public void rollOut(double speed) {
        intakeKraken.set(-Math.abs(speed));
    }

    public void roll(double speed) {
        intakeKraken.set(speed);
    } 

    public void stop() {
        intakeKraken.stopMotor();
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }
}