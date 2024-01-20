package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsytem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this PivotSubsytemSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static PivotSubsytem INSTANCE = new PivotSubsytem();

    /**
     * Returns the Singleton instance of this PivotSubsytemSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code PivotSubsytemSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static PivotSubsytem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this PivotSubsytemSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private PivotSubsytem() {
    }
}

