package frc.lib.exceptions;

import java.io.FileNotFoundException;

public class FileInvalidException extends FileNotFoundException {

    private static final String defaultException = "Unable to access file resource";
    public static final long serialVersionUID = 1000112656;
    private String customMessage;

    public FileInvalidException(String filepath) {
        customMessage = filepath;
    }

    public String getMessage() {
        return customMessage;
    }

    public String toString() {
        return defaultException + " : " + customMessage;
    }
}