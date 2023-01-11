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

import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class SocketTableServer {

    // Number of threads to handle incoming requests
    protected static final int CLIENT_POOL_SIZE = 4;

    private int port = 7777;

    protected boolean stopped = false;

    private SocketTableData socketTableData = new SocketTableData();

    public SocketTableServer() {
    }

    public SocketTableServer(int port) {
        this.port = port;
    }

    public void stop() {
        this.stopped = true;
    }

    public void start() {
        final ExecutorService clientProcessingPool = Executors.newFixedThreadPool(CLIENT_POOL_SIZE);

        Runnable serverTask = () -> {

            ServerSocket serverSocket = null;
            try {
                // Bind server socket
                serverSocket = new ServerSocket(SocketTableServer.this.port);
                System.out.println("SocketTableServer is listening on port: " + SocketTableServer.this.port);
            } catch (Exception ex) {
                SocketTableServer.this.stopped = true;
                System.out.println("Failed to start SocketTableServer on port: " + SocketTableServer.this.port);
                ex.printStackTrace();
            }

            // Continuously listen for incoming socket connections
            while (serverSocket != null && !SocketTableServer.this.stopped) {
                Socket clientSocket = null;

                try {
                    // Accept an incoming socket connection
                    clientSocket = serverSocket.accept();
                } catch (Exception ex) {
                    System.out.println("Failure on serverSocket accept.");
                    ex.printStackTrace();
                }

                // Handle the socket message and response
                if (clientSocket != null) {
                    clientProcessingPool.submit(new SocketTableMessageHandler(socketTableData, clientSocket));
                }
            }

            System.out.println("SocketTableServer stopped.");
        };

        // Start the SocketTableServer
        Thread serverThread = new Thread(serverTask);
        serverThread.start();
    }

    public boolean isStopped() {
        return stopped;
    }

    public int getPort() {
        return port;
    }

    public SocketTableData getData() {
        return this.socketTableData;
    }

    public static void main(String[] args) {
        SocketTableServer server = new SocketTableServer();
        server.start();
    }

}
