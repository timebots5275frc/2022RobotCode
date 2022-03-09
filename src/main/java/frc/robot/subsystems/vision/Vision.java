// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */
    public Vision() {
        this.run();
    }

    public void run() {
        // get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // get a reference to the subtable called "datatable"
        NetworkTable datatable = inst.getTable("GRIP");

        // get a reference to key in "datatable" called "Y"
        NetworkTableEntry yEntry = datatable.getEntry("Y");
        inst.startClientTeam(190);

        // add an entry listener for changed values of "X", the lambda ("->" operator)
        // defines the code that should run when "X" changes
        datatable.addEntryListener("X", (table, key, entry, value, flags) -> {
            System.out.println("X changed value: " + value.getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // add an entry listener for changed values of "Y", the lambda ("->" operator)
        // defines the code that should run when "Y" changes
        yEntry.addListener(event -> {
            System.out.println("Y changed value: " + event.getEntry().getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        /*
         * try {
         * Thread.sleep(10000);
         * } catch (InterruptedException ex) {
         * System.out.println("Interrupted");
         * Thread.currentThread().interrupt();
         * return;
         * }
         */
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }
}
