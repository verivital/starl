package edu.illinois.mitra.starlSim.main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.AbstractAction;
import javax.swing.InputMap;
import javax.swing.KeyStroke;

public class MotionAction extends AbstractAction implements ActionListener{

    public MotionAction(String name){
        super(name);

    }

    public void actionPerformed(ActionEvent move){
        System.out.println("Action Performed");

    }

    public MotionAction addAction(String name){
        MotionAction action = new MotionAction(name);
        KeyStroke pressedKeyStroke = KeyStroke.getKeyStroke(name);
        //InputMap inputMap = component.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
       // inputMap.put(pressedKeyStroke, name);
       // component.getActionMap().put(name, action);

        return action;
    }


}
