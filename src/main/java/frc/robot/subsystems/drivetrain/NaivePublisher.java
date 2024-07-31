package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.function.Consumer;

public class NaivePublisher<T> {
    private final ArrayList<NaiveSubscriber<T>> subscribers = new ArrayList<>();

    public void subscribe(NaiveSubscriber<T> subscriber) {
         subscribers.add(subscriber);
    }

    public void cancel(NaiveSubscriber<T> subscriber) {
        subscribers.remove(subscriber);
    }

    public void publish(T publishedItem) {
        for (NaiveSubscriber<T> subscriber : subscribers) {
            subscriber.publishNext(publishedItem);
        }
    }

    public static class NaiveSubscriber<O> {
        private final Consumer<O> consumer;

        public NaiveSubscriber(Consumer<O> consumer) {
            this.consumer = consumer;
        }

        protected void publishNext(O publishedItem) {
            this.consumer.accept(publishedItem);
        }
    }
}
