package org.a05annex.frc.subsystems;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * This is a test class built to read data from april tag tracking runs with
 * the {@link org.a05annex.frc.subsystems.SpeedCachedSwerve} and help tune the prediction algorithms of the
 * {@link org.a05annex.frc.subsystems.SpeedCachedSwerve}
 */
public class TuneSpeedCache  extends JFrame implements ActionListener, WindowListener {

    public static void main(@NotNull final String[] args) {
        // Setup the commandline argument parser and parse any commandline arguments
        ArgumentParser parser = ArgumentParsers.newFor("TuneSpeedCache").build()
                .description("Speed cache tuner");
        parser.addArgument("-a", "--aprilTag")
                .type(String.class)
                .help("specify april tag log file");
        parser.addArgument("-s", "--swerve")
                .type(String.class)
                .help("specify a swerve log file");
        String aprilTagFile = null;
        String swerveFile = null;
        try {
            Namespace parsedArgs = parser.parseArgs(args);
            aprilTagFile = parsedArgs.get("aprilTag");
            swerveFile = parsedArgs.get("swerve");
        } catch (ArgumentParserException e) {
            parser.handleError(e);
        }
        // start display window
        try {
            final TuneSpeedCache tuneSpeedCache = new TuneSpeedCache(aprilTagFile, swerveFile);
            tuneSpeedCache.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Creating a test SpeedCachedSwerve
    // -----------------------------------------------------------------------------------------------------------------
    //These are the configuration settings for the 2023 competition robot - might want to move these to a data file
    static A05Constants.RobotSettings robotSettings = new A05Constants.RobotSettings(0, "Competition",
            0.5461, 0.5461, 2.700, 1.161,
            2.723, 2.448, 1.026,0.9650);

    /**
     * The length of the speed cache for this test - let's guess we will not collect more than 60 seconds of test data
     * a about 50 frames/sec (20ms per frame). 50fps * 60sec = 3000frames, use 1000 to give us a safety factor of 3.3
     */
    static private final int TEST_CACHE_LENGTH = 10000;

    static SpeedCachedSwerve getInitializedSCS() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        A05Constants.RobotSettings cc = robotSettings;
        SCS.setDriveGeometry(cc.length, cc.width,
                cc.rf, cc.rr, cc.lf, cc.lr,
                cc.maxSpeedCalibration);
        SCS.setCacheLength(TEST_CACHE_LENGTH);
        return SCS;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the main TuneSpeedCache instance that controls the application window.
    // -----------------------------------------------------------------------------------------------------------------
    static final int APRIL_TIME_INDEX = 0;
    static final int APRIL_DISTANCE_INDEX = 1;
    static final int APRIL_STRAFE_INDEX = 2;
    static final int SPEED_CACHE_DISTANCE_INDEX = 3;
    static final int SPEED_CACHE_STRAFE_INDEX = 4;

    static final int SWERVE_TIME_INDEX = 0;
    static final int SWERVE_SPEED_INDEX = 1;
    static final int SWERVE_STRAFE_INDEX = 2;
    static final int SWERVE_ROTATE_INDEX = 3;

    static class ColumnStats {
        final String name;
        double min = Double.NaN;
        double max = Double.NaN;
        double ave = 0.0;
        double minDelta = Double.NaN;
        double maxDelta = Double.NaN;
        double aveDelta = 0.0;
        ColumnStats(String name) {
            this.name = name;
        }
    }

    int nSizeX = 700;
    int nSizeY = 500;
    private final GraphicsConfiguration graphicsConfig;   // the graphics configuration of the window device

    List<List<Double>> aprilTagData = null;
    List<ColumnStats> aprilTagStats = new ArrayList<>();
    List<List<Double>> swerveData = null;
    List<ColumnStats> swerveStats = new ArrayList<>();

    SpeedCachedSwerve speedCachedSwerve = getInitializedSCS();

    private final TuneSpeedCacheCanvas canvas;

    TuneSpeedCache( String aprilTagFile, String swerveFile) {
        //-----------------------------------------------------------------------------------------------
        // setup the window for drawing the field and paths
        //-----------------------------------------------------------------------------------------------
        // resize the default if it doesn't fit in the full screen
        graphicsConfig = getGraphicsConfiguration();
        final Rectangle boundsRect = graphicsConfig.getBounds();
        if (boundsRect.width < nSizeX) nSizeX = boundsRect.width;
        if (boundsRect.height < nSizeY) nSizeY = boundsRect.height;
        setSize(nSizeX, nSizeY);
        //-----------------------------------------------------------------------------------------------
        // Get the logged april tag and speed cache data files, load the SpeedCacheSwerve
        //-----------------------------------------------------------------------------------------------
        aprilTagData = readCsvFile(aprilTagFile, aprilTagStats, APRIL_TIME_INDEX);
        swerveData = readCsvFile(swerveFile, swerveStats, SWERVE_TIME_INDEX);
        for (List<Double> swerveCommand : swerveData) {
            speedCachedSwerve.addControlRequest(swerveCommand.get(SWERVE_SPEED_INDEX),
                    swerveCommand.get(SWERVE_STRAFE_INDEX), swerveCommand.get(SWERVE_ROTATE_INDEX),
                    swerveCommand.get(SWERVE_TIME_INDEX));
        }
        //-----------------------------------------------------------------------------------------------
        // create a canvas to draw on
        //-----------------------------------------------------------------------------------------------
        canvas = new TuneSpeedCacheCanvas(graphicsConfig, aprilTagData, aprilTagStats, swerveData,
                swerveStats, speedCachedSwerve);

        add(canvas, BorderLayout.CENTER);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        try {
            Desktop desktop = Desktop.getDesktop();
            desktop.setQuitHandler((e, r) -> exitTuneSpeedCache());
        } catch (UnsupportedOperationException e) {
            System.out.println("No desktop quit handler setup, not supported by this platform.");
        }


    }

    List<List<Double>> readCsvFile(String filename, List<ColumnStats> columnStats, int timeIndex) {
        List<List<Double>> records = new ArrayList<>();
        int lineCt = -1;
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            List<Double> lastRecord = null;
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                if (-1 != lineCt) {
                    // skip the header, only save the data
                    List<Double> record =  new ArrayList<>();
                    int valueInd = 0;
                    for (String strValue : values) {
                        Double value = Double.valueOf(strValue);
                        record.add(value);
                        ColumnStats stats = columnStats.get(valueInd);
                        if (Double.isNaN(stats.min) || (value < stats.min)) {
                            stats.min = value;
                        }
                        if (Double.isNaN(stats.max) || (value > stats.max)) {
                            stats.max = value;
                        }
                        stats.ave += value;
                        if (null != lastRecord) {
                            Double delta = value - lastRecord.get(valueInd);
                            if (Double.isNaN(stats.minDelta) || (delta < stats.minDelta)) {
                                stats.minDelta = delta;
                            }
                            if (Double.isNaN(stats.maxDelta) || (delta > stats.maxDelta)) {
                                stats.maxDelta = delta;
                            }
                            stats.aveDelta += delta;
                        }
                        valueInd++;
                    }
                    records.add(record);
                    lastRecord = record;
                } else {
                    // create the stats for this data set
                    for (String name : values) {
                        columnStats.add(new ColumnStats(name));
                    }
                }
                lineCt++;
            }
            for (ColumnStats stats : columnStats) {
                stats.ave /= lineCt;
                stats.aveDelta /= lineCt;
            }
        } catch (IOException e) {
            System.out.println("Error reading file: \"" + filename + "\"");
            throw new RuntimeException(e);
        }
        System.out.println("\"" + filename + "\" read, " + lineCt + " lines of data.");
        return records;
    }

    private void exitTuneSpeedCache() {
        dispose();
    }

    //------------------------------------------------------------------------------------------------------------------
    // ************** ActionListener **************
    @Override
    public void actionPerformed(ActionEvent e) {

    }

    @Override
    public void windowOpened(WindowEvent e) {

    }

    @Override
    public void windowClosing(WindowEvent e) {
        exitTuneSpeedCache();
    }

    @Override
    public void windowClosed(WindowEvent e) {

    }

    @Override
    public void windowIconified(WindowEvent e) {

    }

    @Override
    public void windowDeiconified(WindowEvent e) {

    }

    @Override
    public void windowActivated(WindowEvent e) {

    }

    @Override
    public void windowDeactivated(WindowEvent e) {

    }

}
