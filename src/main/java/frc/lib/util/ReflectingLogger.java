package frc.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.exceptions.DirectoryNotFoundException;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.ParameterizedType;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class ReflectingLogger<T> {

    protected static final String dataSeperator = ", ";

    protected PrintWriter output = null;
    protected Map<Field, T> classFieldMap = new LinkedHashMap<>();
    protected final DecimalFormat fmt = new DecimalFormat("#0.000");

    protected ReflectingLogger() {

    }

    /**
     * M
     * 
     * @param dataClasses
     * @throws FileNotFoundException
     */
    public ReflectingLogger(List<T> dataClasses) throws FileNotFoundException {
        this(dataClasses, getMount("robotdata"), false);
    }

    /**
     * constructor that allows the specific file to be set and avoid using the
     * default mount
     * 
     * @param dataClasses the list of data classes to log on
     * @param loggingFile the file to log to
     */
    public ReflectingLogger(List<T> dataClasses, File loggingFile) {
        this(dataClasses, loggingFile, false);
    }

    /**
     * broadest constructor for the reflecting logger
     * 
     * @param dataClasses  the list of data classes to log on
     * @param loggingFile  the file to log to
     * @param allowRethrow whether the logger is allowed to throw a runtime error if
     *                     the file cannot be opened
     */
    public ReflectingLogger(List<T> dataClasses, File loggingFile, Boolean allowRethrow) {
        // generate map of subsystem IO's and fields
        for (T dataClass : dataClasses) {
            if (dataClass == null)
                continue;
            for (Field field : dataClass.getClass().getFields()) {
                classFieldMap.put(field, dataClass);
            }
        }

        // create the base file and header
        generateHeader(loggingFile, allowRethrow);
    }

    /**
     * Function for generating and writing the CSV header into the specified logging
     * file
     * 
     * @param loggingFile  the file to generate the header into and to use for
     *                     logging
     * @param allowRethrow whether the logger should allow throwing a runtime error
     *                     if the file cannot be opened
     */
    protected void generateHeader(File loggingFile, boolean allowRethrow) {
        try {
            output = new PrintWriter(loggingFile.getAbsolutePath());

            // Write field names
            StringBuffer line = new StringBuffer();
            line.append("time_double");
            for (Map.Entry<Field, T> entry : classFieldMap.entrySet()) {
                line.append(dataSeperator);
                String name = entry.getKey().getName();
                try {
                    if (CSVWritable.class.isAssignableFrom(entry.getKey().getType())) {
                        for (int i = 0; i < ((CSVWritable) entry.getKey().get(entry.getValue())).getNumFields(); i++) {
                            if (i != 0)
                                line.append(dataSeperator);
                            line.append(name + "_" + entry.getKey().getType().getSimpleName() + "_" + i);
                        }
                    } else if (entry.getKey().getType().isArray()) {
                        final int len = Array.getLength(entry.getKey().get(entry.getValue()));
                        for (int i = 0; i < len; i++) {
                            if (i != 0)
                                line.append(dataSeperator);
                            line.append(name + "_" + entry.getKey().getType().getSimpleName() + "_" + i + "_" + len);
                        }
                    } else if (List.class.isAssignableFrom(entry.getKey().getType())) {
                        line.append(name + "_list_" + ((Class<?>)((ParameterizedType)entry.getKey().getGenericType()).getActualTypeArguments()[0]).getSimpleName());
                    } else {
                        line.append(name + "_" + entry.getKey().getType().getSimpleName());
                    }
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    line.append(name + "_" + entry.getKey().getType().getSimpleName());
                }
            }

            // Write the first line of the file
            writeLine(line.toString());
        } catch (FileNotFoundException e) {
            // allows the code not to boot if logging functionality cannot start
            if (allowRethrow) {
                throw new RuntimeException(e.getCause());
            }

            // otherwise kick out the stack trace for the error and stop the logger
            e.printStackTrace();
        }
    }

    /**
     * Function that writes the next update to the log file. It reads a list of data
     * classes and dumps their contents into the opened file
     * 
     * @param dataClasses   the list of data classes to log
     * @param fpgaTimestamp the current FPGA time of the system to record
     */
    public void update(List<T> dataClasses, double fpgaTimestamp) {

        // generate map of subsystem IO's and fields
        for (T dataClass : dataClasses) {
            if (dataClass == null)
                continue;
            for (Field field : dataClass.getClass().getFields()) {
                classFieldMap.put(field, dataClass);
            }
        }

        logMap(fpgaTimestamp);

    }

    /**
     * Internal function for taking the class field map and writing it to the opened
     * file
     * 
     * @param fpgaTimestamp, the curent timestamp to write in for the file update
     */
    protected void logMap(double fpgaTimestamp) {

        // no writer avaliable to update exit the update
        if (output.equals(null))
            return;

        final StringBuffer line = new StringBuffer();

        // Append starting time
        line.append(fmt.format(fpgaTimestamp));

        // for all fields in map generate
        for (Map.Entry<Field, T> entry : classFieldMap.entrySet()) {
            // append separator
            line.append(dataSeperator);

            // this shouldnt happen but is here for safety reasons
            if (entry == null)
                continue;

            // Attempt to append subsystem IO value
            try {
                if (entry.getKey().get(entry.getValue()) == null) {
                    line.append("null");
                } else if (entry.getKey().getType().isArray()) {
                    final int len = Array.getLength(entry.getKey().get(entry.getValue()));
                    for (int i = 0; i < len; i++) {
                        if (i != 0)
                            line.append(dataSeperator);
                        line.append(Array.get(entry.getKey().get(entry.getValue()), i));
                    }
                } else if (List.class.isAssignableFrom(entry.getKey().getType())) {
                    for (int i = 0; i < ((List<Object>) entry.getKey().get(entry.getValue())).size(); i++) {
                        if (i != 0)
                            line.append(dataSeperator);
                        final Object current = ((List<Object>) entry.getKey().get(entry.getValue())).get(i);
                        if (current != null)
                            line.append(current);
                    }
                } else if (CSVWritable.class.isAssignableFrom(entry.getKey().getType())) {
                    line.append(((CSVWritable) entry.getKey().get(entry.getValue())).toCSV());
                } else {
                    line.append(entry.getKey().get(entry.getValue()).toString());
                }
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        writeLine(line.toString());

    }

    /**
     * method to write a line to the currently open file
     * 
     * @param line the complied line of text to write and flush to the file
     */
    protected synchronized void writeLine(String line) {
        if (output != null) {
            output.println(line);
            output.flush();
        }
    }

    public synchronized void close() {
        if (output != null) {
            output.flush();
            output.close();
            classFieldMap.clear();
        }
    }

    /**
     * A function to scan for a logging folder inside a usb device mounted to the
     * roborio. There is a kernel issue where external drive folders will remain
     * mounted in the file system after a device has been removed
     * 
     * @param fileName general name of the file to generate with
     * @return a file reference to the created logging file in the usb drive's
     *         logging folder
     * @throws FileNotFoundException if a logging directory is not found a
     *                               DirectoryNotFoundException will be thrown
     */
    public static File getMount(String fileName) throws FileNotFoundException {
        if (RobotBase.isReal()) {
            // create base file reference looking for the media directory
            File media = new File("/media");
            if (!media.exists()) {
                throw new DirectoryNotFoundException("/media");
            }

            if (media.listFiles().length < 1) {
                throw new DirectoryNotFoundException("No media devices found in system");
            }

            // Locate the currently active media drive by finding a nested logging directory
            File logging_path = null;
            for (File mount : media.listFiles()) {
                logging_path = new File(mount.getAbsolutePath() + "/logging");
                if (logging_path.isDirectory()) {
                    System.out.println(logging_path.getAbsolutePath());
                    break;
                }
                logging_path = null;
            }

            if (logging_path.equals(null)) {
                throw new DirectoryNotFoundException("No media device with a logging directory was found");
            }

            File fileref = new File(logging_path.getAbsolutePath() + File.separator + getTimeStampedFileName(fileName));
            return fileref;
        } else {
            Path path = Paths.get(Filesystem.getLaunchDirectory() + File.separator + "src" + File.separator + "main"
                    + File.separator + "sim");
            if (!Files.exists(path)) {
                try {
                    Files.createDirectories(path);
                } catch (Exception e) {
                    throw new DirectoryNotFoundException(path.toString());
                }

            }
            return new File(path.toString() + File.separator + getTimeStampedFileName(fileName));
        }

    }

    /**
     * Generates a new file name tagged with the creation time and the overall file
     * name
     * 
     * @param fileName the base filename to use that will have the timestamp and
     *                 extension appended to it
     */
    protected static String getTimeStampedFileName(String fileName) {
        SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
        outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        String newDateString = outputFormatter.format(new Date());
        return fileName + "_" + newDateString + "_LOG.csv";
    }

}
