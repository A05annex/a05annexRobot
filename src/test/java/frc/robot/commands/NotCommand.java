package frc.robot.commands;

/**
 * This is a class that does not extend {@link edu.wpi.first.wpilibj2.command.Command}, but
 * will be instantiated in an autonomous path test as though it is a command. This is to
 * test that the error handling works as expected.
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
