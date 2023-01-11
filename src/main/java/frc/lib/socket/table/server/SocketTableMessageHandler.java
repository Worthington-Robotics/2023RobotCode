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

import frc.lib.socket.table.util.MessageParser;
import frc.lib.socket.table.util.RequestType;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Map;

public class SocketTableMessageHandler implements Runnable {

    private static final int PACKET_SIZE = 1024;

    private final Socket clientSocket;

    private final SocketTableData socketTableData;

    public SocketTableMessageHandler(SocketTableData socketTableData, Socket clientSocket) {
        this.socketTableData = socketTableData;
        this.clientSocket = clientSocket;
    }

    @Override
    public void run() {
        // System.out.println("Client connected: " + this.clientSocket.toString());
        try {
            // Create reader/write
            PrintWriter out = new PrintWriter(this.clientSocket.getOutputStream(), true);
            BufferedReader in = new BufferedReader(new InputStreamReader(this.clientSocket.getInputStream()));

            // Read incoming message
            char[] input = new char[PACKET_SIZE];
            in.read(input, 0, PACKET_SIZE);
            String message = new String(input);

            // System.out.println("Message received: " + message);

            // Parse message and generate response
            String response = null;
            if (message != null) {
                response = handleMessage(message);
            }

            // Send response
            // System.out.println("Sending Response: " + response);
            out.println(response);

        } catch (Exception ex) {
            System.out.println("Failure processing SocketTableServer request.");
            ex.printStackTrace();
        } finally {
            closeSocket();
        }
    }

    protected String handleMessage(String message) {

        String response = null;
        String request = MessageParser.parseMessage(message, MessageParser.REQUEST_PATTERN);
        String key = MessageParser.parseMessage(message, MessageParser.KEY_PATTERN);

        if (request != null) {
            if (request.equals(RequestType.GETALL.toString())) {
                // Handle GETALL
                Map<String, String> data = socketTableData.getAll();
                response = MessageParser.formatGetAllResponse(data);
            } else {
                String result = null;

                if (key != null && request.equals(RequestType.GET.toString())) {
                    // Handle GET
                    result = socketTableData.getString(key, null);
                } else if (key != null && request.equals(RequestType.UPDATE.toString())) {
                    // Handle UPDATE
                    String value = MessageParser.parseMessage(message, MessageParser.VALUE_PATTERN);
                    result = socketTableData.updateString(key, value);
                } else if (key != null && request.equals(RequestType.DELETE.toString())) {
                    // Handle DELETE
                    result = socketTableData.delete(key);
                } else {
                    System.out.println("Unknown request type: " + request);
                }

                // TODO: Handle callbacks
                response = MessageParser.formatResponse(key, result);
            }
        } else {
            System.out.println("Unable to parse message: " + message);
        }
        return response;
    }

    private void closeSocket() {
        if (clientSocket != null) {
            try {
                this.clientSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

}
