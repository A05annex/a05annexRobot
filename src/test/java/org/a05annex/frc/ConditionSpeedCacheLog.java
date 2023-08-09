package org.a05annex.frc;

import org.jetbrains.annotations.NotNull;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This program parses a log file downloaded from the robot after a test robot run with adn April target to
 * extract data useful for tuning the speed cache position prediction algorithms. When this program is run it
 * generates 3 data files:
 * <ul>
 *     <li><b>./scrubbedRawData_<i>date</i>.csv:</b> - the records from the data file that are relevant.</li>
 *     <li><b>./AprilData_<i>date</i>.csv:</b> - the data to plot the robot position as reported by the
 *     april tag vision processing, and the position predicted by the speed cache at the time of the
 *     test (since we are tuning the speed cache, this is just a snapshot of how the speed cache performed
 *     on the date of the test.</li>
 *     <li><b>./SwerveData_<i>date</i>.csv:</b> - the swerve drive data to load the speed cache for
 *     tuning/testing.</li>
 * </ul>
 * Run this program with 2 arguments:
 * <ul>
 *     <li><b>Raw Data Filename:</b> The name of the raw data file downloaded, such as:
 *       <i>/Users/roy/Downloads/SpeedCacheRaw_20230712.csv</i></li>
 *     <li><b>date:</b> the date used in the generated filenames. Preferred format is <i>yyyymmdd</i></li>
 * </ul>
 */
public class ConditionSpeedCacheLog {

    static int hasValidFrameIndex = -1;
    static int aprilTagTimeIndex = -1;
    static int aprilTagStrafeIndex = -1;
    static int aprilTagDistIndex = -1;
    static int cumulativeCacheStrafeIndex = -1;
    static int cumulativeCacheDistIndex = -1;
    static int swerveTimeIndex = -1;
    static int swerveSpeedIndex = -1;
    static int swerveDirectionIndex = -1;
    static int swerveRotateIndex = -1;

    public static void main(@NotNull final String[] args) {
        if (2 != args.length) {
            System.out.println("Hey, you; that's right you trying to run this program...\n" +
                    "     The command line arguments are: datafile date\n" +
                    "Please read the program docs before you do this again!!");
            System.exit(-1);
        }

        // Initially, we did this as a:
        //   * read the entire file into memory
        //   * process the recodes to pull out the key april target and swerve command entries
        //   * write the key april target and swerve command entries
        // But - we realized that this could take huge memory because there is so much redundant and
        // unnecessary stuff in the log file, so, we reworked it to totally process each record as it
        // is read so that the only data in memory is the data required to write the scrubbed raw data
        // file and the conditioned files for testing/tuning the speed cache.
        //
        List<List<String>> meaningfulRawRecords = new ArrayList<>();
        List<List<String>> meaningfulAprilRecords = new ArrayList<>();
        meaningfulAprilRecords.add(Arrays.asList("aprilTime", "aprilTimeDelta", "aprilDistance", "aprilStrafe",
                "predictedDistance", "predictedStrafe"));
        String lastAprilTimeStr = "null";
        double lastAprilTime = 0.0;
        String thisAprilTimeStr = "null";
        double aprilTimeDelta = 0.0;
        List<List<String>> meaningfulSwerveRecords = new ArrayList<>();
        meaningfulSwerveRecords.add(Arrays.asList("swerveTime", "swerveTimeDelta", "speed", "direction", "rotate"));
        String thisSwerveTimeStr = "null";
        double lastSwerveTime = 0.0;
        String lastSwerveTimeStr = "null";
        double swerveTimeDelta = 0.0;
        List<String> lastRecord = null;
        int lineCt = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(args[0]))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                List<String> record = Arrays.asList(values);
                if (0 == lineCt) {
                    meaningfulRawRecords.add(record);
                    mapHeader(record);
                } else {
                    if (0 == record.size()) {
                        // this is an empty record, skip it
                        continue;
                    }
                    boolean recordIsImportant = false;
                    thisAprilTimeStr = record.get(aprilTagTimeIndex);
                    if (!lastAprilTimeStr.equals(thisAprilTimeStr) && !thisAprilTimeStr.equals("null")) {
                        // This is good, a new april tag time on this line - the last record was the time and location
                        // that we want to log; i.e. it may be an important point.
                        double thisAprilTime = Double.parseDouble(thisAprilTimeStr);
                        if (!lastAprilTimeStr.equals("null") && (thisAprilTime > lastAprilTime)) {
                            //the last record was the record we should use for the april tag data
                            meaningfulAprilRecords.add(Arrays.asList(
                                    formatDouble("%.4f",lastRecord.get(aprilTagTimeIndex)),
                                    String.format("%.4f",aprilTimeDelta),
                                    formatDouble("%.5f",lastRecord.get(aprilTagDistIndex)),
                                    formatDouble("%.5f",lastRecord.get(aprilTagStrafeIndex)),
                                    formatDouble("%.5f",lastRecord.get(cumulativeCacheDistIndex)),
                                    formatDouble("%.5f",lastRecord.get(cumulativeCacheStrafeIndex))));
                            aprilTimeDelta = thisAprilTime - lastAprilTime;
                            recordIsImportant = true;
                        }
                        lastAprilTimeStr = thisAprilTimeStr;
                        lastAprilTime = Double.parseDouble(lastAprilTimeStr);
                    }
                    thisSwerveTimeStr = record.get(swerveTimeIndex);
                    if (!lastSwerveTimeStr.equals(thisSwerveTimeStr)) {
                        // This is good, a new april tag time on this line - the last line was the time and location
                        // that we want to log; i.e. it may be an important point.
                        double thisSwerveTime = Double.parseDouble(thisSwerveTimeStr);
                        if (!lastSwerveTimeStr.equals("null") && (thisSwerveTime > lastSwerveTime)) {
                            //the last record was the record we should use for the april tag data
                            meaningfulSwerveRecords.add(Arrays.asList(
                                    formatDouble("%.4f",lastRecord.get(swerveTimeIndex)),
                                    String.format("%.4f",swerveTimeDelta),
                                    formatDouble("%.5f",lastRecord.get(swerveSpeedIndex)),
                                    formatDouble("%.5f",lastRecord.get(swerveDirectionIndex)),
                                    formatDouble("%.5f",lastRecord.get(swerveRotateIndex))));
                            swerveTimeDelta = thisSwerveTime - lastSwerveTime;
                            recordIsImportant = true;
                        }
                        lastSwerveTimeStr = thisSwerveTimeStr;
                        lastSwerveTime = Double.parseDouble(lastSwerveTimeStr);
                    }
                    if (recordIsImportant) {
                        meaningfulRawRecords.add(record);
                    }
                    lastRecord = record;
                }
                lineCt++;
            }
        } catch (IOException e) {
            System.out.println("Error reading file: \"" + args[0] + "\"");
            throw new RuntimeException(e);
        }

        writeCSV("./scrubbedRawData_" + args[1] + ".csv", meaningfulRawRecords);
        writeCSV("./AprilData_" + args[1] + ".csv", meaningfulAprilRecords);
        writeCSV("./SwerveData_" + args[1] + ".csv", meaningfulSwerveRecords);
        System.exit(0);
    }

    static void mapHeader(List<String> headerRecord) {
        // There is a lot of useless crap in the log files - i'e' rows that duplicate almost everything
        // except 1 value, and empty rows. So let's sort out which lines actually mean something
        // figure out which are important
        int i = 0;
        for (String header : headerRecord) {
            switch (header) {
                case "actualPositionTimeLog":
                    aprilTagTimeIndex = i;
                    break;
                case "actualXPosition":
                    aprilTagDistIndex = i;
                    break;
                case "actualYPosition":
                    aprilTagStrafeIndex = i;
                    break;
                case "cacheXPosition":
                    cumulativeCacheDistIndex = i;
                    break;
                case "cacheYPosition":
                    cumulativeCacheStrafeIndex = i;
                    break;
                case "swerveTime":
                    swerveTimeIndex = i;
                    break;
                case "speed":
                    swerveSpeedIndex = i;
                    break;
                case "direction":
                    swerveDirectionIndex = i;
                    break;
                case "rotate":
                    swerveRotateIndex = i;
                    break;

            }
            i++;
        }
    }

    /**
     *
     * @param filename
     * @param csvData
     */
    static void writeCSV(String filename, List<List<String>> csvData) {
        File csvOutputFile = new File(filename);
        try (PrintWriter pw = new PrintWriter(csvOutputFile)) {
            csvData.stream()
                    .map(ConditionSpeedCacheLog::convertRowToCSV)
                    .forEach(pw::println);
        } catch (IOException e) {
            System.out.println("Error writing file: " + filename);
            throw new RuntimeException(e);
        }
    }

    static String formatDouble(String format, String doubleAsString) {
        return String.format(format , Double.parseDouble(doubleAsString));
    }

    static String convertRowToCSV(List<String> data) {
        return data.stream()
                .map(ConditionSpeedCacheLog::escapeSpecialCharacters)
                .collect(Collectors.joining(","));
    }

    static String escapeSpecialCharacters(String data) {
        String escapedData = data.replaceAll("\\R", " ");
        if (data.contains(",") || data.contains("\"") || data.contains("'")) {
            data = data.replace("\"", "\"\"");
            escapedData = "\"" + data + "\"";
        }
        return escapedData;
    }


}
