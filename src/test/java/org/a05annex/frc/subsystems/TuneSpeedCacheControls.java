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

    TuneSpeedCacheControls(TuneSpeedCacheCanvas canvas) {
        super(new BorderLayout(5, 5));
        this.canvas = canvas;
        JPanel controlPanel = new JPanel(new GridLayout(0, 1, 2, 2));
        pkgLoadAndAddLabel(controlPanel, "Displayed Data:");
        for (TuneSpeedCacheCanvas.PlottedPath plottedPath : canvas.plottedPaths) {
            plottedPathCheckbox.add(pkgLoadAndAddCheckbox(controlPanel, plottedPath.name, plottedPath.displayed));
        }
        controlPanel.add(new JSeparator(SwingConstants.HORIZONTAL));
        pkgLoadAndAddLabel(controlPanel, "Speed Cache Tuning:");


        pkgLoadAndAddLabel(controlPanel, "Swerve Max Velocity:");
        JSlider speedScale = new JSlider(JSlider.HORIZONTAL,
                900, 1100, 1000);
        speedScale.addChangeListener(this);
        //Turn on labels at major tick marks.
        speedScale.setMajorTickSpacing(100);
        speedScale.setMinorTickSpacing(10);
        speedScale.setPaintTicks(true);
        Hashtable labelTable = new Hashtable();
        labelTable.put( Integer.valueOf( 900), new JLabel("90%") );
        labelTable.put( Integer.valueOf( 1000 ), new JLabel("100%") );
        labelTable.put( Integer.valueOf( 1100 ), new JLabel("110%") );
        speedScale.setLabelTable( labelTable );
        speedScale.setPaintLabels(true);
        controlPanel.add(speedScale);
        
        pkgLoadAndAddLabel(controlPanel, "Phase:");
        JSlider phaseSlider = new JSlider(JSlider.HORIZONTAL,
                0, 1000, 500);
        phaseSlider.addChangeListener(this);
        //Turn on labels at major tick marks.
        phaseSlider.setMajorTickSpacing(500);
        phaseSlider.setMinorTickSpacing(50);
        phaseSlider.setPaintTicks(true);
        Hashtable phaseLabelTable = new Hashtable();
        phaseLabelTable.put( Integer.valueOf( 0), new JLabel("0.0") );
        phaseLabelTable.put( Integer.valueOf( 500 ), new JLabel("0.5") );
        phaseLabelTable.put( Integer.valueOf( 1000 ), new JLabel("1.0") );
        phaseSlider.setLabelTable( phaseLabelTable );
        phaseSlider.setPaintLabels(true);
        controlPanel.add(phaseSlider);
        
        

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

    // ------------------------------ ItemListener start ---------------------------------------------------------------
    // This listener deals with changes in the checkboxes
    @Override
    public void itemStateChanged(ItemEvent e) {
        Object item = e.getItem();
        int plottedPathIndex = 0;
        for (JCheckBox checkbox : plottedPathCheckbox) {
            if (item == checkbox) {
               if (e.getStateChange() == ItemEvent.SELECTED) {
                   canvas.plottedPaths.get(plottedPathIndex).displayed = true;
               } else if (e.getStateChange() == ItemEvent.DESELECTED){
                   canvas.plottedPaths.get(plottedPathIndex).displayed = false;
               }
               canvas.repaint();
               break;
            }
            plottedPathIndex++;
        }
    }
    // ------------------------------ ItemListener end -----------------------------------------------------------------

    // ------------------------------ ChangeListener start -------------------------------------------------------------
    // This listener deals with changes in the sliders (max swerve speed, phase)
    @Override
    public void stateChanged(ChangeEvent e) {

    }
    // ------------------------------ ChangeListener end ---------------------------------------------------------------
}


