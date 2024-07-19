package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.ElevatorPositions;
import frc.robot.constants.TalonFXConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor;

    private final DigitalInput magSwitch;
    private ElevatorConstants.ElevatorPositions elevatorPositions =
            ElevatorConstants.ElevatorPositions.DOWN;

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID, TalonFXConstants.CANIVORE_NAME);
        magSwitch = new DigitalInput(9);
        var ElConfigurator = elevatorMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.CurrentLimits = ElevatorConstants.ELEVATOR_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = ElevatorConstants.ELEVATOR_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = ElevatorConstants.SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.PROFILE_V;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.PROFILE_A;

        elevatorMotor
                .getRotorVelocity()
                .waitForUpdate(ElevatorConstants.ELEVATOR_VELOCITY_STATUS_FRAME);
        elevatorMotor
                .getRotorPosition()
                .waitForUpdate(ElevatorConstants.ELEVATOR_POSITION_STATUS_FRAME);

        ElConfigurator.apply(talonFXConfiguration);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void goUp(double speed) {
        elevatorMotor.set(Math.abs(speed));
    }

    public void goDown(double speed) {
        elevatorMotor.set(-Math.abs(speed));
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    public boolean getmagSwitch() {
        return magSwitch.get();
    }

    public double getEncoder() {
        return elevatorMotor.getRotorPosition().getValue();
    }

    public void setRotations(double rotations) {
        elevatorMotor.setPosition(rotations);
    }

    // why do we have two of these methods????

    public void setPosition(ElevatorConstants.ElevatorPositions position) {
        elevatorPositions = position;

        MotionMagicExpoVoltage motionMagicVoltage =
                new MotionMagicExpoVoltage(position.distanceEl, true, 0.0, 0, true, false, false);

        elevatorMotor.setControl(motionMagicVoltage.withPosition(position.distanceEl));
    }

    public void setPosition2(ElevatorConstants.ElevatorPositions position) {
        elevatorPositions = position;

        MotionMagicExpoVoltage motionMagicVoltage =
                new MotionMagicExpoVoltage(position.distanceEl, true, 4.0, 0, true, false, false);

        elevatorMotor.setControl(
                motionMagicVoltage.withPosition(position.distanceEl).withFeedForward(10));
    }

    @Override
    public void periodic() {
        if (getmagSwitch() && elevatorPositions == ElevatorPositions.DOWN) {
            setRotations(0);
        }
        SmartDashboard.putData("Elevator motor", elevatorMotor);
        SmartDashboard.putData("Elevator magswitch", magSwitch);
    }

    public Command positionDownCmd() {
        return runOnce(() -> setPosition(ElevatorConstants.ElevatorPositions.DOWN));
    }
}
