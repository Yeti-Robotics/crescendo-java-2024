package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class RobotDataPublisher<T> {
    private final List<RobotDataSubscription<T>> subscriptions = new ArrayList<>();

    public <R> RobotDataPublisher<R> map(Function<T, R> mapper) {
        RobotDataPublisher<R> newPublisher = new RobotDataPublisher<>();
        subscribeWith(data -> newPublisher.publish(mapper.apply(data))).start();
        return newPublisher;
    }

    public void publish(T data) {
        for (RobotDataSubscription<T> subscription : subscriptions) {
            subscription.publishNext(data);
        }
    }

    public RobotDataSubscription<T> subscribeWith(Consumer<T> consumer) {
        return new RobotDataSubscription<>(this, consumer);
    }

    public static class RobotDataSubscription<O> {
        private final RobotDataPublisher<? extends O> robotDataPublisher;
        private final Consumer<O> consumer;
        private boolean accept = false;

        private RobotDataSubscription(RobotDataPublisher<O> robotDataPublisher, Consumer<O> consumer) {
            this.robotDataPublisher = robotDataPublisher;
            this.consumer = consumer;
            robotDataPublisher.subscriptions.add(this);
        }

        private void publishNext(O data) {
            if (accept) {
                consumer.accept(data);
            }
        }

        public void start() {
            accept = true;
        }

        public void cancel() {
            robotDataPublisher.subscriptions.remove(this);
        }
    }
}
