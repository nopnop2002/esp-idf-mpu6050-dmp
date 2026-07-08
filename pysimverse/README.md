# Drone simulator using pysimverse
You can pilot the drone using the ESP32.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/b1297dbc-e92b-4c18-9f93-2ca9d04a883c" />

# Install pysimverse
pysimverse runs on Windows or Mac.   
Get the pysimverse installer for your OS from [here](https://pysimverse.com/).   
The operating system must be restarted after installation.   

# Start pysimverse
Select missions.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/500c8e09-99d5-47c6-8e61-d66c26ec5ea8" />

Start missions.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/76bc1d43-5fcc-4ab2-a4b3-eec76005c023" />

The drone is waiting to start its flight.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/4b3b5dd0-283e-4f85-b60d-b08b80e678e6" />

# Start python script
Python script works as a UDP server.   
Pilot the drone by obtaining IMU attitude data via UDP.   
```
+-------------+          +-------------+          +-------------+                 +-------------+
|             |          |             |          |             |                 |             |
|     IMU     |--(ic2)-->|    ESP32    |--(UDP)-->|   main.py   |--(RC control)-->| pysimverse  |
|             |          |             |          |             |                 |             |
+-------------+          +-------------+          +-------------+                 +-------------+
```

```
$ pip install pysimverse
$ pip install keyboard
$ cd esp-idf-mpu6050-dmp/pysimverse
$ python main.py
```

When the script starts, the drone takes off.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/b1297dbc-e92b-4c18-9f93-2ca9d04a883c" />

You control the drone by tilting the IMU.   
<img width="1244" height="677" alt="Image" src="https://github.com/user-attachments/assets/c049b19a-9d8d-4b09-a729-c067b5be28d7" />

Press the Esc key or the q key to land the drone.   

# python API
https://github.com/Otr437/PYTHON_DRONE_SIMVERSE   
