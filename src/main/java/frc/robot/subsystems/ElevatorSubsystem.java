package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.ElevatorPositions;
import frc.robot.constants.TalonFXConstants;

public class ElevatorSubsystem {
    private final TalonFX elevatorMotor;
    private final CANcoder elevatorCoder;
    private ElevatorConstants.ElevatorPositions elevatorPositions = ElevatorConstants.ElevatorPositions.DOWN;

    public ElevatorSubsystem(){
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID, TalonFXConstants.CANIVORE_NAME);
        elevatorCoder = new CANcoder(ElevatorConstants.ELEVATOR_CAN_ID);

        var ElConfigurator = elevatorMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.CurrentLimits = ElevatorConstants.ELEVATOR_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = ElevatorConstants.ELEVATOR_SOFT_LIMIT;

        elevatorMotor.getRotorVelocity().waitForUpdate(ElevatorConstants.ELEVATOR_VELOCITY_STATUS_FRAME);
        elevatorMotor.getRotorPosition().waitForUpdate(ElevatorConstants.ELEVATOR_POSITION_STATUS_FRAME);

        ElConfigurator.apply(talonFXConfiguration);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);


        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = ElevatorConstants.ELEVATOR_F;
        slot0Configs.kP = ElevatorConstants.ELEVATOR_P;
        slot0Configs.kI = ElevatorConstants.ELEVATOR_I;
        slot0Configs.kD = ElevatorConstants.ELEVATOR_D;
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

    public void SetPosition(ElevatorConstants.ElevatorPositions position){
        elevatorPositions = position;

        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
                position.sensorUnitsEl, true, 0.0 , 0,
                true, false, false);

        elevatorMotor.setControl(motionMagicVoltage);
    }


}