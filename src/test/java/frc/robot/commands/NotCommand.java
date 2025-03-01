package frc.robot.commands;

/**
 * This is a class that does not extend {@link edu.wpi.first.wpilibj2.command.Command}, but
 * will be instantiated in a test as though it is a command.
 */
public class NotCommand {
    public static int instantiationCt = 0;

    public NotCommand() {
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (NotCommand.class) {
            instantiationCt += 1;
        }
    }
}
