package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.*;

public class ElevatorSubsystem {
    private final TalonFX elevatorMotor;
    private final CANcoder elevatorCoder;

    public ElevatorSubsystem(){
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID);
        elevatorCoder = new CANcoder(ElevatorConstants.ELEVATOR_CAN_ID);

        var ElConfigurator = elevatorMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.CurrentLimits = ElevatorConstants.ELEVATOR_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = ElevatorConstants.ELEVATOR_SOFT_LIMIT;

        elevatorMotor.getRotorVelocity().waitForUpdate(ElevatorConstants.ELEVATOR_VELOCITY_STATUS_FRAME);
        elevatorMotor.getRotorPosition().waitForUpdate(ElevatorConstants.ELEVATOR_POSITION_STATUS_FRAME);

        ElConfigurator.apply(talonFXConfiguration);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void goUp(double sped){
        elevatorMotor.set(Math.abs(sped));
    }

    public void goDown(double sped){
        elevatorMotor.set(-Math.abs(sped));
    }

    public void stop(){
        elevatorMotor.stopMotor();
    }


}