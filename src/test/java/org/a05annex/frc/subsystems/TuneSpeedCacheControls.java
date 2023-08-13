package org.a05annex.frc.subsystems;

import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.*;
import java.util.List;

public class TuneSpeedCacheControls extends JPanel implements ItemListener {

    final TuneSpeedCacheCanvas canvas;
    List<JCheckBox> plottedPathCheckbox = new ArrayList<JCheckBox>();

    TuneSpeedCacheControls(TuneSpeedCacheCanvas canvas) {
        super(new BorderLayout(5, 5));
        this.canvas = canvas;
        JPanel displayPanel = new JPanel(new GridLayout(canvas.plottedPaths.size() + 1, 1, 2, 2));
        JLabel labelX = pkgLoadAndAddLabel(displayPanel, "Displayed Data");
        for (TuneSpeedCacheCanvas.PlottedPath plottedPath : canvas.plottedPaths) {
            plottedPathCheckbox.add(pkgLoadAndAddCheckbox(displayPanel, plottedPath.name, plottedPath.displayed));
        }
        add(displayPanel, BorderLayout.LINE_START);
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
}
