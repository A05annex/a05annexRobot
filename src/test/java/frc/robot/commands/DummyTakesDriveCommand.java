package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.a05annex.frc.commands.ICanTakeDrive;

public class DummyTakesDriveCommand extends Command implements ICanTakeDrive {

    // --------------------------------------------------------------------------------------------
    // These are some static parameters set during the run of the command, and examined only for
    // testing, to assure the command has operated as expected.
    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static int canTakeDriveFailCt;
    public static int executeCt = 0;
    public static int requestedCyclesBeforeTakeDrive;
    public static int requestedCyclesToTarget;

    static public void zeroCounts() {
        instantiationCt = 0;
        initializationCt = 0;
        endCt = 0;
        canTakeDriveFailCt = 0;
        executeCt = 0;
        requestedCyclesBeforeTakeDrive = 0;
        requestedCyclesToTarget = 0;
    }
    // --------------------------------------------------------------------------------------------

    static final int DEFAULT_CYCLES_BEFORE_TAKE_DRIVE = 0;
    static final int DEFAULT_CYCLES_TO_TARGET = 50;
    final int cyclesBeforeTakeDrive;
    final int cyclesToTarget;

    boolean hasTakenDrive = false;
    int cycleCt = 0;

    /**
     * Instantiate a dummy command that takes immediate control of the drive
     * and runs for 50 cycles
     */
    public DummyTakesDriveCommand() {
        this.cyclesBeforeTakeDrive = DEFAULT_CYCLES_BEFORE_TAKE_DRIVE;
        this.cyclesToTarget = DEFAULT_CYCLES_TO_TARGET;
        System.out.printf("**** Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (DummyTakesDriveCommand.class) {
            instantiationCt += 1;
            requestedCyclesBeforeTakeDrive += this.cyclesBeforeTakeDrive;
            requestedCyclesToTarget += this.cyclesToTarget;
        }
    }

    /**
     * Instantiate a dummy command that takes control of the drive after
     * the specified {@code cyclesBeforeTakeDrive}, and runs for a total
     * {@code cyclesToTarget} before reaching the target (this includes the {@code cyclesBeforeTakeDrive}.
     *
     * @param cyclesBeforeTakeDrive The number of command cycles before this command says it can
     *                              take control of the drive.
     * @param cyclesToTarget The number of command cycles between when this command says it can take the
     *                       drive and when it says it is finished.
     */
    public DummyTakesDriveCommand(Integer cyclesBeforeTakeDrive, Integer cyclesToTarget) {
        this.cyclesBeforeTakeDrive = cyclesBeforeTakeDrive;
        this.cyclesToTarget = cyclesToTarget;
        System.out.printf("**** Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (DummyTakesDriveCommand.class) {
            instantiationCt += 1;
            requestedCyclesBeforeTakeDrive += this.cyclesBeforeTakeDrive;
            requestedCyclesToTarget += this.cyclesToTarget;
        }
    }
    @Override
    public boolean canTakeDrive() {
        if (cycleCt < cyclesBeforeTakeDrive) {
            synchronized (DummyTakesDriveCommand.class) {
                canTakeDriveFailCt += 1;
            }
            cycleCt++;
            System.out.printf("false = %s.canTakeDrive(), ct = %d%n", this.getClass().getName(), cycleCt);
            return false;
        }
        hasTakenDrive = true;
        System.out.printf("true = %s.canTakeDrive()%n", this.getClass().getName());
        return true;
    }
    @Override
    public void initialize() {
        synchronized (DummyTakesDriveCommand.class) {
            initializationCt += 1;
        }

    }
    @Override
    public void execute() {
        synchronized (DummyTakesDriveCommand.class) {
            executeCt += 1;
        }
        cycleCt++;
        System.out.printf("%s.execute(), ct = %d%n", this.getClass().getName(), cycleCt);
    }

    @Override
    public boolean isFinished() {
        return cycleCt >= cyclesToTarget;
    }

    @Override
    public void end(boolean interrupted) {
        synchronized (DummyTakesDriveCommand.class) {
            endCt++;
            if (interrupted && !hasTakenDrive) {
                requestedCyclesBeforeTakeDrive -= cyclesBeforeTakeDrive - cycleCt;
                requestedCyclesToTarget -= cyclesBeforeTakeDrive - cycleCt;
            }
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
