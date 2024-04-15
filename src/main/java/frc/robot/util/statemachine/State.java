package frc.robot.util.statemachine;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class State {
    public String description = "";
    private StateMachineManager stateMachineManager;
    private Command onStateEnter;

    public State(String description, Command onStateEnter) {
        this.description = description;
        this.onStateEnter = onStateEnter.beforeStarting(new InstantCommand(() -> stateMachineManager.setCurrentState(this)));
    }

    public void setStateMachineManager(StateMachineManager stateMachineManager) {
        this.stateMachineManager = stateMachineManager;
    }

    public Command getOnStateEnter() {
        return onStateEnter;
    }

    public void withTransitionEventOnSignal(BooleanSupplier signal, State endState) {
        BooleanEvent event = new BooleanEvent(stateMachineManager.getLoop(), signal)
            .and(() -> stateMachineManager.getCurrentState() == this);
        event.castTo(Trigger::new).onTrue(endState.onStateEnter);
    }

    public void withTransitionEventOnCommandComplete(State endState) {
        BooleanEvent event = new BooleanEvent(stateMachineManager.getLoop(), onStateEnter::isScheduled).falling()
            .and(() -> stateMachineManager.getCurrentState() == this);
        event.castTo(Trigger::new).onTrue(endState.onStateEnter);
    }
}
