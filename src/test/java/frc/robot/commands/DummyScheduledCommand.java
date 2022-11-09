package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This is a dummy scheduled command. It is a dummy in the sense that it does not actually do anything except
 * run 10 execute steps and finish, and count both the number of times it was instantiated and the number of
 * times {@link #execute()} runs> This is for post-test verification that this command was instantiated the
 * expected number of times and ran the expected number of command cycles.
 */
public class DummyScheduledCommand extends CommandBase {

    public static final int EXECUTES_PER_SCHEDULED_RUN = 10;
    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static int executeCt = 0;


    int m_executeCt = 0;
    public DummyScheduledCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (DummyScheduledCommand.class) {
            instantiationCt += 1;
        }
    }

    @Override
    public void initialize() {
        synchronized (DummyScheduledCommand.class) {
            initializationCt += 1;
        }

    }

    @Override
    public void execute() {
        synchronized (DummyScheduledCommand.class) {
            executeCt += 1;
        }
        m_executeCt += 1;
    }

    @Override
    public boolean isFinished() {
        return m_executeCt >= EXECUTES_PER_SCHEDULED_RUN;
    }

    @Override
    public void end(boolean interrupted) {
        synchronized (DummyScheduledCommand.class) {
            endCt++;
        }
    }

    /**
     * return true so we can test this thing.
     * @return {@code true}
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
