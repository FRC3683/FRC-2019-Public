/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AutoSelector {

    

    public static class StartingPosition {

        private String name;
        private List<NameCommandPair> commands;
        private int index;

        private static class NameCommandPair{
            private String name;
            private Command command;
            
            public NameCommandPair(String name, Command command){
                this.name = name;
                this.command = command;
            }

            public String getName(){
                return name;
            }

            public Command getCommand(){
                return command;
            }
        }

        public StartingPosition(String positionName) {
            this.name = positionName;
            commands = new ArrayList<>();
            index = -1;
        }

        public int size(){
            return commands.size();
        }

        public StartingPosition addCommand(String name, Command command) {
            if(commands.isEmpty()){
                index = 0;
            }
            commands.add(new NameCommandPair(name, command));
            return this;
        }

        public StartingPosition addDefaultCommand(String name, Command command) {            
            addCommand(name, command);
            index = commands.size() - 1;
            return this;
        }

        public StartingPosition next(){
            index++;
            index %= commands.size();
            return this;
        }

        public StartingPosition previous(){
            index--;
            index %= commands.size();
            return this;
        }

        public Command getActive(){
            return commands.get(index).getCommand();
        }
    }

    private List<StartingPosition> startingPos;
    private int index;

    NetworkTableInstance instance;
    NetworkTable table;
    NetworkTableEntry startingPositionEntry;
    NetworkTableEntry autoActionEntry;

    public AutoSelector() {
        startingPos = new ArrayList<>();
        index = -1;
        
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("Robot");
        startingPositionEntry = table.getEntry("Starting Position");
        autoActionEntry = table.getEntry("Auto Action");
    }

    public StartingPosition addStartingPos(String positionName) {
        StartingPosition result = new StartingPosition(positionName);
        if(startingPos.isEmpty()){
            index = 0;
        }
        startingPos.add(result);
        return result;
    }

    public StartingPosition addDefaultStartingPos(String positionName) {
        StartingPosition result = addStartingPos(positionName);
        index = startingPos.size() - 1;
        return result;
    }

    public void next(){
        index++;
        index %= startingPos.size();
    }

    public void previous(){
        index--;
        index %= startingPos.size();
    }

    // need code to listen to selection changes for starting position,
    // and respond by updating the smartDashboard with the corresponding
    // command chooser.
    // maybe need a "refresh" method on this class to poll the selected
    // starting position and then update the command chooser.

    public Command getActive() {
        return startingPos.get(index).getActive();
    }
}
