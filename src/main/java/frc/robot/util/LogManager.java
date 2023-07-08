package frc.robot.util;

import java.util.HashMap;


import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (https://github.com/Mechanical-Advantage/AdvantageScope)
 */
public class LogManager {

  private static DataLog log = DataLogManager.getLog();



  // These are the log entries that are not updated periodically, they just receive individual values.
  private static HashMap<String, DoubleLogEntry> individualDoubleLogs = new HashMap<>();
  private static HashMap<String, DoubleArrayLogEntry> individualDoubleArrayLogs = new HashMap<>();
  private static HashMap<String, BooleanLogEntry> individualBooleanLogs = new HashMap<>();
  private static HashMap<String, IntegerLogEntry> individualIntegerLogs = new HashMap<>();

  /**
   * Records the metadata supplied by gversion (https://github.com/lessthanoptimal/gversion-plugin) in BuildData.java.
   */
  public static void recordMetadata() {
    new StringLogEntry(log, "BuildData/Maven Group").append(BuildData.MAVEN_GROUP);
    new StringLogEntry(log, "BuildData/Maven Name").append(BuildData.MAVEN_NAME); // The name of the repository
    new StringLogEntry(log, "BuildData/Version").append(BuildData.VERSION);
    new IntegerLogEntry(log, "BuildData/Git Revision").append(BuildData.GIT_REVISION);
    new StringLogEntry(log, "BuildData/Git SHA").append(BuildData.GIT_SHA); // The SHA code for the latest commit
    new StringLogEntry(log, "BuildData/Git date").append(BuildData.GIT_DATE);
    new StringLogEntry(log, "BuildData/Git Branch").append(BuildData.GIT_BRANCH); // The branch name
    new StringLogEntry(log, "BuildData/Build Date").append(BuildData.BUILD_DATE);
    new IntegerLogEntry(log, "BuildData/Build Unix Time").append(BuildData.BUILD_UNIX_TIME);
    new IntegerLogEntry(log, "BuildData/Dirty").append(BuildData.DIRTY);
  }


  /**
   * Logs a single double value to the log. Do not use with the other addDouble() that takes a double supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the double supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addDouble(String name, double value) {
    if (individualDoubleLogs.containsKey(name)) {
      individualDoubleLogs.get(name).append(value);
    } else {
      individualDoubleLogs.put(name, new DoubleLogEntry(log, name));
      individualDoubleLogs.get(name).append(value);
    }
  }


  /**
   * Logs a single double array to the log. Do not use with the other addDoubleArray() that takes a double array supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the double array supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addDoubleArray(String name, double[] value) {
    if (individualDoubleArrayLogs.containsKey(name)) {
      individualDoubleArrayLogs.get(name).append(value);
    } else {
      individualDoubleArrayLogs.put(name, new DoubleArrayLogEntry(log, name));
      individualDoubleArrayLogs.get(name).append(value);
    }
  }


  /**
   * Logs a single int to the log. Do not use with the other addInt() that takes a int supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the int supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addInt(String name, int value) {
    if (individualIntegerLogs.containsKey(name)) {
      individualIntegerLogs.get(name).append(value);
    } else {
      individualIntegerLogs.put(name, new IntegerLogEntry(log, name));
      individualIntegerLogs.get(name).append(value);
    }
  }


  /**
   * Logs a single boolean to the log. Do not use with the other addBoolean() that takes a boolean supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the boolean supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addBoolean(String name, boolean value) {
    if (individualBooleanLogs.containsKey(name)) {
      individualBooleanLogs.get(name).append(value);
    } else {
      individualBooleanLogs.put(name, new BooleanLogEntry(log, name));
      individualBooleanLogs.get(name).append(value);
    }
  }

  /**
   * Logs all the values that have been collected. Should be called periodically. 
   */
 
}