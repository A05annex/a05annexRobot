package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.a05annex.frc.commands.TestAutonomousPathCommand;


/**
 *
 */
public class DummyStopAndRunCommand extends Command {

    public static long DEFAULT_STOP_AND_RUN_DURATION = 2000;

    // --------------------------------------------------------------------------------------------
    // These are some static parameters set during the run of the command, and examined only for
    // testing, to assure the command has operated as expected.
    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static long requestedStopAndRunDuration = 0;
    public static long stopAndRunDuration = 0;
    public static long expectedDurationTolerance = 0;
    public static int executeCt = 0;
    static public void zeroCounts() {
        instantiationCt = 0;
        initializationCt = 0;
        endCt = 0;
        requestedStopAndRunDuration = 0;
        stopAndRunDuration = 0;
        expectedDurationTolerance = 0;
        executeCt = 0;
    }
    // --------------------------------------------------------------------------------------------


    final private long startTime = System.currentTimeMillis();
    final private long endTime;

    /**
     * The no-argument constructor uses a default duration of 2.0 seconds
     */
    public DummyStopAndRunCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        endTime = startTime + DEFAULT_STOP_AND_RUN_DURATION - (TestAutonomousPathCommand.COMMAND_CYCLE_TIME_MS / 2);
        System.out.printf("**** Instantiating command: class='%s'%n", this.getClass().getName());
        System.out.printf("****          '%s':  ends at %d%n", this.getClass().getName(), endTime);
        synchronized (DummyStopAndRunCommand.class) {
            instantiationCt += 1;
            requestedStopAndRunDuration += DEFAULT_STOP_AND_RUN_DURATION;
            expectedDurationTolerance += (TestAutonomousPathCommand.COMMAND_CYCLE_TIME_MS / 2);
        }
    }
    public DummyStopAndRunCommand(Double duration) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        long msDuration = (long)(duration * 1000.0);
        endTime = startTime + msDuration - (TestAutonomousPathCommand.COMMAND_CYCLE_TIME_MS / 2);
        System.out.printf("**** Instantiating command: class='%s'%n", this.getClass().getName());
        System.out.printf("****          '%s':  ends at %d%n", this.getClass().getName(), endTime);
        synchronized (DummyStopAndRunCommand.class) {
            instantiationCt += 1;
            requestedStopAndRunDuration += msDuration;
            expectedDurationTolerance += (TestAutonomousPathCommand.COMMAND_CYCLE_TIME_MS / 2);
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
        return (System.currentTimeMillis() > endTime);
    }

    @Override
    public void end(boolean interrupted) {
        long actualEndTime = System.currentTimeMillis();
        long actualDuration = actualEndTime - startTime;
        System.out.printf("%n          '%s':  ends after %dms%n", this.getClass().getName(),
                actualDuration);
        synchronized (DummyStopAndRunCommand.class) {
            endCt++;
            stopAndRunDuration += actualDuration;
        }
    }
}
