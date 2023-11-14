// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** Add your docs here. */
public class LogOrDash {
    private static DataLog l;

    private static HashMap<String, DoubleLogEntry> doubles;
    private static HashMap<String, BooleanLogEntry> booleans;
    private static HashMap<String, StringLogEntry> strings;

    public static boolean sendToDash;

    public static void setupLogging()
    {
        // Start data logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        l = DataLogManager.getLog();

        doubles = new HashMap<String, DoubleLogEntry>();
        booleans = new HashMap<String, BooleanLogEntry>();
        strings = new HashMap<String, StringLogEntry>();

        sendToDash = true;

        
    }

    public static void logNumber(String key, double value)
    {
        if(sendToDash)
        {
            SmartDashboard.putNumber(key, value);
        }
        else
        {
            DoubleLogEntry d = doubles.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new DoubleLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                doubles.put(key, d);
            }
        }
    }

    
    public static void logBoolean(String key, boolean value)
    {
        if(sendToDash)
        {
            SmartDashboard.putBoolean(key, value);
        }
        else
        {
            BooleanLogEntry d = booleans.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new BooleanLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                booleans.put(key, d);
            }
        }
    }

    
    public static void logString(String key, String value)
    {
        if(sendToDash)
        {
            SmartDashboard.putString(key, value);
        }
        else
        {
            StringLogEntry d = strings.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new StringLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                strings.put(key, d);
            }
        }
    }

    public static WrapperCommand toggleDashboard()
    {
        return new InstantCommand(() -> {
            sendToDash = !sendToDash;
        }).ignoringDisable(true);
    }
}
