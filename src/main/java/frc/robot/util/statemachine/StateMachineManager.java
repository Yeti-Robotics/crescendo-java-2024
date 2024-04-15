package frc.robot.util.statemachine;

import edu.wpi.first.wpilibj.event.EventLoop;

public class StateMachineManager {
    private EventLoop loop;
    private State currentState;

    public StateMachineManager(State... states) {
       loop = new EventLoop();

       for (State s : states) {
           s.setStateMachineManager(this);
       }
    }

    public void update() {
        loop.poll();
    }

    public EventLoop getLoop() {
        return loop;
    }

    public State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(State currentState) {
        this.currentState = currentState;
    }
}
