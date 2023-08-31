package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

public class StatisticsUtil {

  private static double[] doubleListToArray(List<Double> arrayList){
    return arrayList.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public static double mean(List<Double> data){
    return mean(doubleListToArray(data));
  }

  public static double mean(double... data){
    double mean = 0;
    for (double datum : data) {
      mean += datum;
    }
    mean /= data.length;
    return mean;
  }

  public static double stdDev(List<Double> data){
    return stdDev(doubleListToArray(data));
  }

  public static double stdDev(double... data){
    if (data.length == 0 || data.length == 1) return 0;
    
    double mean = mean(data);
    
    double total = 0;
    for (double datum : data) {
      total += Math.pow(datum - mean, 2);
    }
    return Math.sqrt(total / (data.length-1));
  }
}
