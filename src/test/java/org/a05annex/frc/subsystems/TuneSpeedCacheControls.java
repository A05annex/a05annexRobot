package org.a05annex.frc.subsystems;

import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.*;
import java.util.List;

public class TuneSpeedCacheControls extends JPanel implements ItemListener, ChangeListener {

    final TuneSpeedCacheCanvas canvas;
    List<JCheckBox> plottedPathCheckbox = new ArrayList<JCheckBox>();
    List<JCheckBox> plottedTestCheckbox = new ArrayList<JCheckBox>();
    JSlider forwardScaleSlider = null;
    JSlider strafeScaleSlider = null;
    JSlider phaseSlider = null;
    JSlider offsetSlider = null;

    TuneSpeedCacheControls(TuneSpeedCacheCanvas canvas) {
        super(new BorderLayout(5, 5));
        this.canvas = canvas;
        JPanel controlPanel = new JPanel(new GridLayout(0, 1, 2, 2));
        pkgLoadAndAddLabel(controlPanel, "Displayed Test:");
        int testId = 1;
        for (TuneSpeedCacheCanvas.PlottedTest plottedTest : canvas.plottedPaths ) {
            plottedTestCheckbox.add(pkgLoadAndAddCheckbox(controlPanel, "test %d".formatted(testId),
                    plottedTest.isDisplayed));
            testId++;
        }
        controlPanel.add(new JSeparator(SwingConstants.HORIZONTAL));
        pkgLoadAndAddLabel(controlPanel, "Displayed Paths:");
        for (TuneSpeedCacheCanvas.PlottedPath plottedPath : canvas.plottedPaths.get(0).plottedPaths) {
            plottedPathCheckbox.add(pkgLoadAndAddCheckbox(controlPanel, plottedPath.name, plottedPath.displayed));
        }
        controlPanel.add(new JSeparator(SwingConstants.HORIZONTAL));
        pkgLoadAndAddLabel(controlPanel, "Speed Cache Tuning:");


        Hashtable labelTable = new Hashtable();
        labelTable.put( Integer.valueOf( 800), new JLabel("80%") );
        //labelTable.put( Integer.valueOf( 900), new JLabel("90%") );
        labelTable.put( Integer.valueOf( 1000 ), new JLabel("100%") );
        //labelTable.put( Integer.valueOf( 1100 ), new JLabel("110%") );
        labelTable.put( Integer.valueOf( 1200 ), new JLabel("120%") );
        pkgLoadAndAddLabel(controlPanel, "Forward Velocity Scale:");
        forwardScaleSlider = phgLoadAndAddSlider(controlPanel, 700, 1300,
                (int)(1000.0 * canvas.speedCachedSwerve.getMaxForwardScale()),
                100,25, labelTable );
        pkgLoadAndAddLabel(controlPanel, "Strafe Velocity Scale:");
        strafeScaleSlider = phgLoadAndAddSlider(controlPanel, 700, 1300,
                (int)(1000.0 * canvas.speedCachedSwerve.getMaxForwardScale()),
                100,25, labelTable );

        pkgLoadAndAddLabel(controlPanel, "Phase:");
        Hashtable<Integer, JLabel> phaseLabelTable = new Hashtable<>();
        phaseLabelTable.put(0, new JLabel("0.0") );
        phaseLabelTable.put(500, new JLabel("0.5") );
        phaseLabelTable.put(1000, new JLabel("1.0") );
        phaseSlider = phgLoadAndAddSlider(controlPanel, 0, 1000,
                (int)(500.0 * canvas.speedCachedSwerve.getPhase()),
                500,50, phaseLabelTable );

        pkgLoadAndAddLabel(controlPanel, "April Offset:");
        Hashtable<Integer, JLabel> offsetLabelTable = new Hashtable<>();
        offsetLabelTable.put(-1600, new JLabel("-160ms") );
        offsetLabelTable.put(0, new JLabel("0ms") );
        offsetLabelTable.put(1600, new JLabel("160ms") );
        offsetSlider = phgLoadAndAddSlider(controlPanel, -1600, 1600,
                0,200,100, offsetLabelTable );

        add(controlPanel, BorderLayout.LINE_START);
    }

    private @NotNull JLabel pkgLoadAndAddLabel(@NotNull JPanel panel, String name) {
        JLabel label = new JLabel(name, SwingConstants.LEADING);
        panel.add(label);
        return label;
    }
    private @NotNull JCheckBox pkgLoadAndAddCheckbox(@NotNull JPanel panel, String name, boolean initialState) {
        JCheckBox checkBox = new JCheckBox(name);
        // checkBox.setMnemonic(KeyEvent.VK_G);
        checkBox.setSelected(initialState);
        panel.add(checkBox);
        checkBox.addItemListener(this);
        return checkBox;
    }

    private @NotNull JSlider phgLoadAndAddSlider(@NotNull JPanel panel, int min, int max, int value,
                                                 int majorTic, int minorTic, Hashtable labels) {
        JSlider slider = new JSlider(JSlider.HORIZONTAL,
                min, max, value);
        slider.addChangeListener(this);
        //Turn on labels at major tick marks.
        slider.setMajorTickSpacing(majorTic);
        slider.setMinorTickSpacing(minorTic);
        slider.setPaintTicks(true);
        if (null != labels) {
            slider.setLabelTable(labels);
            slider.setPaintLabels(true);
        }
        panel.add(slider);
        return slider;
    }

    // ------------------------------ ItemListener start ---------------------------------------------------------------
    // This listener deals with changes in the checkboxes
    @Override
    public void itemStateChanged(ItemEvent e) {
        Object item = e.getItem();
        int testIndex = 0;
        for (JCheckBox checkbox : plottedTestCheckbox ) {
            if (item == checkbox) {
                if (e.getStateChange() == ItemEvent.SELECTED) {
                    canvas.plottedPaths.get(testIndex).isDisplayed = true;
                } else if (e.getStateChange() == ItemEvent.DESELECTED) {
                    canvas.plottedPaths.get(testIndex).isDisplayed = false;
                }
                canvas.repaint();
                return;
            }
            testIndex++;
        }
        int plottedPathIndex = 0;
        for (JCheckBox checkbox : plottedPathCheckbox) {
            if (item == checkbox) {
                for ( TuneSpeedCacheCanvas.PlottedTest plottedTest : canvas.plottedPaths) {
                    if (e.getStateChange() == ItemEvent.SELECTED) {
                        plottedTest.plottedPaths.get(plottedPathIndex).displayed = true;
                    } else if (e.getStateChange() == ItemEvent.DESELECTED) {
                        plottedTest.plottedPaths.get(plottedPathIndex).displayed = false;
                    }
                }
                canvas.repaint();
                return;
            }
            plottedPathIndex++;
        }
    }
    // ------------------------------ ItemListener end -----------------------------------------------------------------

    // ------------------------------ ChangeListener start -------------------------------------------------------------
    // This listener deals with changes in the sliders (max swerve speed, phase)
    @Override
    public void stateChanged(ChangeEvent e) {
        JSlider source = (JSlider)e.getSource();
        boolean reloadCalcPath = false;
        if (source == forwardScaleSlider) {
            if (source.getValueIsAdjusting()) {
                double scale = (int) source.getValue() / 1000.0;
                System.out.println("forward scale = " + scale);
                canvas.speedCachedSwerve.setMaxForwardScale(scale);
                reloadCalcPath = true;
            }
        }
        else if (source == strafeScaleSlider) {
            if (source.getValueIsAdjusting()) {
                double scale = (int) source.getValue() / 1000.0;
                System.out.println("strafe scale = " + scale);
                canvas.speedCachedSwerve.setMaxStrafeScale(scale);
                reloadCalcPath = true;
            }
        }
        else if (source == phaseSlider) {
            if (source.getValueIsAdjusting()) {
                double phase = (int) source.getValue() / 1000.0;
                System.out.println("phase = " + phase);
                canvas.speedCachedSwerve.setPhase(phase);
                reloadCalcPath = true;
            }
        }
        else if (source == offsetSlider) {
            if (source.getValueIsAdjusting()) {
                double offset = (int) source.getValue() / 10000.0;
                System.out.println("offset = " + offset + "sec");
                canvas.speedCachedSwerve.setLatencyOffset(offset);
                canvas.loadPathFromSelectedPoint();
                canvas.repaint();
            }
        }

        if (reloadCalcPath) {
            int testIndex = 0;
            for (TuneSpeedCacheCanvas.PlottedTest thisTest : canvas.plottedPaths) {
                TuneSpeedCacheCanvas.PlottedPath path = thisTest.
                        plottedPaths.get(TuneSpeedCacheCanvas.SPEED_CACHE_PATH);
                canvas.loadCalculatedSpeedCachePath(canvas.swerveData, canvas.speedCachedSwerve,
                        thisTest. plottedPaths.get(TuneSpeedCacheCanvas.FILTERED_APRIL_TAG_PATH),
                        path);
                path.transformPath(canvas.drawXfm);
                testIndex++;
            }
            canvas.loadPathFromSelectedPoint();
            canvas.repaint();
        }

    }
    // ------------------------------ ChangeListener end ---------------------------------------------------------------
}


