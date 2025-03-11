package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a dummy scheduled command. It is a dummy in the sense that it does not actually do anything except
 * run 10 execute steps and finish, and count both the number of times it was instantiated and the number of
 * times {@link #execute()} runs> This is for post-test verification that this command was instantiated the
 * expected number of times and ran the expected number of command cycles.
 */
public class DummyScheduledCommand extends Command {

    public static final int DEFAULT_EXECUTES_PER_SCHEDULED_RUN = 10;
    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static int requestedExecuteCt = 0;
    public static int executeCt = 0;

    static public void zeroCounts() {
        instantiationCt = 0;
        initializationCt = 0;
        endCt = 0;
        requestedExecuteCt = 0;
        executeCt = 0;
    }


    int m_executeCt = 0;
    int m_maxExecuteCt = DEFAULT_EXECUTES_PER_SCHEDULED_RUN;

    public DummyScheduledCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("**** Instantiating command: class='%s', command cycles=%d%n",
                this.getClass().getName(),m_maxExecuteCt);
        synchronized (DummyScheduledCommand.class) {
            instantiationCt += 1;
            requestedExecuteCt += m_maxExecuteCt;
        }
    }

    public DummyScheduledCommand(Integer cyclesCt) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        m_maxExecuteCt = cyclesCt;
        System.out.printf("**** Instantiating command: class='%s', command cycles=%d%n",
                this.getClass().getName(),m_maxExecuteCt);
        synchronized (DummyScheduledCommand.class) {
            instantiationCt += 1;
            requestedExecuteCt += m_maxExecuteCt;
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
        return m_executeCt >= m_maxExecuteCt;
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
