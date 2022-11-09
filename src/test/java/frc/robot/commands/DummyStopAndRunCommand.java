package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


@SuppressWarnings("unused")
public class DummyStopAndRunCommand extends CommandBase {

    public static final long STOP_AND_RUN_DURATION = 2000;
    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static long stopAndRunDuration = 0;
    public static int executeCt = 0;


    final private long m_startTime = System.currentTimeMillis();
    final private long m_endTime = m_startTime + STOP_AND_RUN_DURATION;

    public DummyStopAndRunCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        System.out.printf("          '%s':  ends at %d%n", this.getClass().getName(), m_endTime);
        synchronized (DummyStopAndRunCommand.class) {
            instantiationCt += 1;
            stopAndRunDuration += STOP_AND_RUN_DURATION;
        }
    }

    @Override
    public void initialize() {
        synchronized (DummyStopAndRunCommand.class) {
            initializationCt += 1;
        }
    }

    @Override
    public void execute() {
        System.out.print(".");
        System.out.flush();
        synchronized (DummyStopAndRunCommand.class) {
            executeCt += 1;
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (System.currentTimeMillis() > m_endTime);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%n          '%s':  ends after %.3f%n", this.getClass().getName(),
                (m_endTime - m_startTime)/1000.0);
        synchronized (DummyStopAndRunCommand.class) {
            endCt++;
        }
    }
}
