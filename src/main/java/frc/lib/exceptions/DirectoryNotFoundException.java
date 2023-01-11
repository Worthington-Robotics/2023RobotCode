package frc.lib.exceptions;

import java.io.FileNotFoundException;

public class DirectoryNotFoundException extends FileNotFoundException {

    private static final String defaultException = "Unable to locate directory";
    public static final long serialVersionUID = 1000112556;
    private String customMessage;


    public DirectoryNotFoundException(String dir) {
        customMessage = dir;
    }

    public String getMessage() {
        return customMessage;
    }

    public String toString() {
        return defaultException + " : " + customMessage;
    }
}