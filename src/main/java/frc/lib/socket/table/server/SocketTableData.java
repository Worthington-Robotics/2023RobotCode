package frc.lib.socket.table.server;

/*
----------------------------------------------------------------------------
Author(s):     	Maxwell Pettit

Date:          	4/1/2019

Description:   	SocketTables provide a socket based communication protocol 
               	for performing simple in-memory CRUD (Create, Read, Update, 
               	Delete) operations. SocketTables are designed to use JSON 
               	messages to provide access to a key-value mapping on a 
               	Java or Python server.
----------------------------------------------------------------------------
*/

import java.util.HashMap;
import java.util.Map;

public class SocketTableData {

    private Map<String, String> data = new HashMap<>();

    public SocketTableData() {
    }

    public synchronized void reset() {
        this.data = new HashMap<>();
    }

    public synchronized String getString(String key, String defaultValue) {
        String value = defaultValue;
        if (data.containsKey(key)) {
            value = this.data.get(key);
        }
        return value;
    }

    public synchronized int getInt(String key, int defaultValue) {
        int value = defaultValue;
        if (this.data.containsKey(key)) {
            String response = data.get(key);
            try {
                value = Integer.parseInt(response);
            } catch (NumberFormatException e) {
                System.out.println("Couldn't parse int.");
            }
        }
        return value;
    }

    public synchronized double getDouble(String key, double defaultValue) {
        double value = defaultValue;
        if (this.data.containsKey(key)) {
            String response = data.get(key);
            try {
                value = Double.parseDouble(response);
            } catch (NumberFormatException e) {
                System.out.println("Couldn't parse double.");
            }
        }
        return value;
    }

    public synchronized boolean getBoolean(String key, boolean defaultValue) {
        boolean value = defaultValue;
        if (this.data.containsKey(key)) {
            String response = data.get(key);
            value = Boolean.parseBoolean(response);
        }
        return value;
    }

    public synchronized String updateString(String key, String value) {
        // TODO: Update timestamps
        this.data.put(key, value);
        return value;
    }

    public synchronized int updateInt(String key, int value) {
        // TODO: Update timestamps
        this.data.put(key, Integer.toString(value));
        return value;
    }

    public synchronized double updateDouble(String key, double value) {
        // TODO: Update timestamps
        this.data.put(key, Double.toString(value));
        return value;
    }

    public synchronized boolean updateBoolean(String key, boolean value) {
        // TODO: Update timestamps
        this.data.put(key, Boolean.toString(value));
        return value;
    }

    public synchronized String delete(String key) {
        String value = null;
        if (this.data.containsKey(key)) {
            value = this.data.remove(key);
        }
        return value;
    }

    public synchronized Map<String, String> getAll() {
        return this.data;
    }

}
