// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static frc.robot.utils.constants.UtilsContants.*;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogManager extends SubsystemBase {

  public static LogManager logManager; // singelton reference

  private DataLog log;
  private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  private NetworkTable table = ntInst.getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  /*
   * class for a single data entry
   */
  public class LogEntry {
    DoubleLogEntry doubleEntry; // wpilib log entry
    BooleanLogEntry booleanEntry;
    @SuppressWarnings("rawtypes")
    StatusSignal phoenix6Status; // supplier of phoenix 6 status signal
    DoubleSupplier getterDouble; // supplier for double data - if no status signal provider
    BooleanSupplier getterBoolean;
    BiConsumer<Double, Long> doubleConsumer = null; // optional consumer when data changed - data value and time
    BiConsumer<Boolean, Long> booleanConsumer = null;
    String name;
    DoublePublisher ntPublisherDouble; // network table punlisher
    BooleanPublisher ntPublisherBoolean;
    double lastValue = Double.MAX_VALUE; // last value - only logging when value changes

    /*
     * Constructor with the suppliers and boolean if add to network table
     */
    @SuppressWarnings("rawtypes")
    LogEntry(String name, StatusSignal phoenix6Status, DoubleSupplier getterDouble, BooleanSupplier getterBoolean,
        boolean addToNT) {

      if (getterBoolean != null || (phoenix6Status != null && phoenix6Status.getTypeClass().equals(Boolean.class))) {
        this.booleanEntry = new BooleanLogEntry(log, name);
      } else {
        if (phoenix6Status != null) {
          LogManager.log(phoenix6Status.getTypeClass());
        }
        this.doubleEntry = new DoubleLogEntry(log, name);
      }
      this.phoenix6Status = phoenix6Status;
      this.getterDouble = getterDouble;
      this.getterBoolean = getterBoolean;
      this.name = name;
      if (addToNT) {
        if (getterBoolean != null || (phoenix6Status != null && phoenix6Status.getTypeClass().equals(Boolean.class))) {
          BooleanTopic bt = table.getBooleanTopic(name);
          ntPublisherBoolean = bt.publish();
        } else {
          DoubleTopic dt = table.getDoubleTopic(name);
          ntPublisherDouble = dt.publish();
        }
      } else {
        ntPublisherDouble = null;
        ntPublisherBoolean = null;
      }
    }

    /*
     * perform a periodic log
     * get the data from the getters and call the actual log
     */
    void log() {
      double v;
      long time = 0;

      if (phoenix6Status != null) {
        var st = phoenix6Status.refresh();
        if (st.getStatus() == StatusCode.OK) {
          v = st.getValueAsDouble();
          time = (long) (st.getTimestamp().getTime() * 1000);
        } else {
          v = 1000000 + st.getStatus().value;
        }
      } else if (getterBoolean != null) {
        v = getterBoolean.getAsBoolean() ? 1 : 0;
        time = 0;
      } else {
        v = getterDouble.getAsDouble();
        time = 0;
      }
      log(v, time);
    }

    /*
     * log a value use zero (current) time
     */
    public void log(double v) {
      log(v, 0);
    }

    /*
     * Log data and time if data changed
     * also publish to network table (if required)
     * also call consumer if set
     */
    public void log(double v, long time) {
      if (v != lastValue) {
        if (getterBoolean != null || (phoenix6Status != null && phoenix6Status.getTypeClass().equals(Boolean.class))) {
          booleanEntry.append(v == 1, time);
          if (ntPublisherBoolean != null) {
            ntPublisherBoolean.set(v == 1);
          }
          if (doubleConsumer != null) {
            doubleConsumer.accept(v, time);
          }
          if (booleanConsumer != null) {
            booleanConsumer.accept(v == 1, time);
          }
        } else  {
          doubleEntry.append(v, time);
          if (ntPublisherDouble != null) {
            ntPublisherDouble.set(v);
          }
          if (doubleConsumer != null) {
            doubleConsumer.accept(v, time);
          }
        }
        lastValue = v;
      }
    }

    // set the consumer
    public void setDoubleConsumer(BiConsumer<Double, Long> consumer) {
      this.doubleConsumer = consumer;
    }

    public void setBooleanConsumer(BiConsumer<Boolean, Long> consumer) {
      this.booleanConsumer = consumer;
    }
  }

  // array of log entries
  ArrayList<LogEntry> logEntries = new ArrayList<>();

  // Log managerconstructor
  public LogManager() {
    logManager = this;

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  /*
   * add a log entry with all data
   */
  @SuppressWarnings("rawtypes")
  private LogEntry add(String name, StatusSignal phoenix6Status, DoubleSupplier getterDouble,
      BooleanSupplier getterBoolean, boolean addToNT) {
    LogEntry entry = new LogEntry(name, phoenix6Status, getterDouble, getterBoolean, addToNT);
    logEntries.add(entry);
    return entry;
  }

  /*
   * get a log entry - if not found, creat one
   */
  private LogEntry get(String name, boolean addToNT) {
    LogEntry e = find(name);
    if (e != null) {
      return e;
    }
    return new LogEntry(name, null, null, null, addToNT);
  }

  /*
   * find a log entry by name
   */
  private LogEntry find(String name) {
    for (LogEntry e : logEntries) {
      if (name.equals(e.name)) {
        return e;
      }
    }
    return null;
  }

  /*
   * Static function - add log entry with all data
   */
  @SuppressWarnings("rawtypes")
  public static LogEntry addEntry(String name, StatusSignal phoenixStatus,
      DoubleSupplier getterDouble, boolean addToNT) {
    return logManager.add(name, phoenixStatus, getterDouble, null, addToNT);
  }

  /*
   * Static function - add log entry for status signal with option to add to
   * network table
   */
  @SuppressWarnings("rawtypes")
  public static LogEntry addEntry(String name, StatusSignal phoenixStatus, boolean addToNT) {
    return logManager.add(name, phoenixStatus, null, null, addToNT);
  }

  /*
   * Static function - add log entry for double supplier with option to add to
   * network table
   */
  public static LogEntry addEntry(String name, DoubleSupplier getterDouble, boolean addToNT) {
    return logManager.add(name, null, getterDouble, null, addToNT);
  }

  /*
   * Static function - add log entry for status signal with network table
   */
  @SuppressWarnings("rawtypes")
  public static LogEntry addEntry(String name, StatusSignal phoenix6Status) {
    return logManager.add(name, phoenix6Status, null, null, true);
  }

  /*
   * Static function - add log entry for double supplier with network table
   */
  public static LogEntry addEntry(String name, DoubleSupplier getterDouble) {
    return logManager.add(name, null, getterDouble, null,  true);
  }

  public static LogEntry addEntry(String name, BooleanSupplier getterBoolean) {
    return logManager.add(name, null, null, getterBoolean, true);
  }

  public static LogEntry addEntry(String name, BooleanSupplier getterBoolean, boolean addToNT) {
    return logManager.add(name, null, null, getterBoolean, addToNT);
  }

  /*
   * Static function - get an entry, create if not foune - will see network table
   * is crating new
   */
  public static LogEntry getEntry(String name, boolean addToNT) {
    return logManager.get(name, addToNT);
  }

  /*
   * Static function - get an entry, create if not foune - will see network table
   * is crating new
   */
  public static LogEntry getEntry(String name) {
    return logManager.get(name, true);
  }

  /*
   * Log text message - also will be sent System.out
   */
  public static ConsoleAlert log(Object message, AlertType alertType) {
    DataLogManager.log(String.valueOf(message));
    
    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message.toString()), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  public static ConsoleAlert log(Object meesage) {
    return log(meesage, AlertType.kInfo);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < activeConsole.size(); i++) {
      if (activeConsole.get(i).isTimerOver()) {
        activeConsole.get(i).close();
        activeConsole.remove(i);
      }
    }

    for (LogEntry e : logEntries) {
      e.log();
    }
  }
}