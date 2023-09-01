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

    static int aprilTagTimeIndex = -1;
    static int aprilTagStrafeIndex = -1;
    static int aprilTagDistIndex = -1;
    static int cumulativeCacheStrafeIndex = -1;
    static int cumulativeCacheDistIndex = -1;
    static int swerveTimeIndex = -1;
    static int swerveSpeedIndex = -1;
    static int swerveDirectionIndex = -1;
    static int swerveRotateIndex = -1;

    static int aprilTagIndex = -1;
    static int speedCacheIndex = -1;

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
        meaningfulRawRecords.add(Arrays.asList("aprilTag","speedCache"));

        List<List<String>> meaningfulAprilRecords = new ArrayList<>();
        meaningfulAprilRecords.add(Arrays.asList("hasTarget", "aprilTime", "aprilDistance", "aprilStrafe",
                "predictedDistance", "predictedStrafe"));
        String lastAprilTagStr = "null";
        String thisAprilTagStr = "null";

        List<List<String>> meaningfulSpeedCacheRecords = new ArrayList<>();
        meaningfulSpeedCacheRecords.add(Arrays.asList("swerveTime", "actualHeading", "expectedHeading",
                "speed", "direction", "rotate"));
        String lastSpeedCacheStr = "null";
        String thisSpeedCacheStr = "null";

        int lineCt = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(args[0]))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                List<String> record = Arrays.asList(values);
                boolean recordIsImportant = false;
                if (0 == lineCt) {
                    mapHeader(record);
                } else {
                    if (0 == record.size()) {
                        // this is an empty record, skip it
                        continue;
                    }
                    thisAprilTagStr = record.get(aprilTagIndex);
                    thisSpeedCacheStr = record.get(speedCacheIndex);
                    if (!thisAprilTagStr.equals("null") && !thisAprilTagStr.equals(lastAprilTagStr)) {
                        String[] aprilTagValues = thisAprilTagStr.replace("\"","").split(";");
                        meaningfulAprilRecords.add(Arrays.asList(aprilTagValues));
                        lastAprilTagStr = thisAprilTagStr;
                        recordIsImportant = true;
                    }
                    if (!thisSpeedCacheStr.equals("null") && !thisSpeedCacheStr.equals(lastSpeedCacheStr) ) {
                        String[] speedCacheValues = thisSpeedCacheStr.replace("\"","").split(";");
                        meaningfulSpeedCacheRecords.add(Arrays.asList(speedCacheValues));
                        lastSpeedCacheStr = thisSpeedCacheStr;
                        recordIsImportant = true;
                    }
                    if (recordIsImportant) {
                        System.out.println(record.get(aprilTagIndex) + "     " + record.get(speedCacheIndex));
                        meaningfulRawRecords.add(record);
                    }
                }
                lineCt++;
            }
        } catch (IOException e) {
            System.out.println("Error reading file: \"" + args[0] + "\"");
            throw new RuntimeException(e);
        }

        writeCSV("./scrubbedRawData_" + args[1] + ".csv", meaningfulRawRecords);
        writeCSV("./AprilData_" + args[1] + ".csv", meaningfulAprilRecords);
        writeCSV("./SwerveData_" + args[1] + ".csv", meaningfulSpeedCacheRecords);
        System.exit(0);
    }

    static void mapHeader(List<String> headerRecord) {
        // There is a lot of useless crap in the log files - i'e' rows that duplicate almost everything
        // except 1 value, and empty rows. So let's sort out which lines actually mean something
        // figure out which are important
        int i = 0;
        for (String header : headerRecord) {
            switch (header) {
                case "speedCache":
                    speedCacheIndex = i;
                    break;
                case "aprilTag":
                    aprilTagIndex = i;
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
