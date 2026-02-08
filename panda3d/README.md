# View Euler angles using panda3d library   
You can view Euler angles using [this](https://www.panda3d.org/) library.   
It works as a UDP display server.   

```
+-------------+          +-------------+          +-------------+
|             |          |             |          |             |
|     IMU     |--(ic2)-->|    ESP32    |--(UDP)-->|  panda.py   |
|             |          |             |          |             |
+-------------+          +-------------+          +-------------+
```

```
$ python3 panda.py --help
usage: panda.py [-h] [--model {jet,biplain,707,fa18}]

options:
  -h, --help            show this help message and exit
  --model {jet,biplain,707,fa18}
```


- using jet model   
	![Image](https://github.com/user-attachments/assets/6d81eec0-5b80-4e5f-ae97-689742253f9a)

- using biplain mode   
	![Image](https://github.com/user-attachments/assets/a37359f2-51f9-439b-88d7-a57bf165265f)

- using 707 mode   
	![Image](https://github.com/user-attachments/assets/a058972d-3708-47f0-b6a0-36f7fbe25c08)

- using fa18 mode   
	![Image](https://github.com/user-attachments/assets/0ed14905-319f-423c-9d79-d5ca66b34f54)

- Move the camera forward and backward   
	Hold down the right button and move the mouse.   

- Move the camera up, down, left and right   
	Hold down the left button and move the mouse.   
	![Image](https://github.com/user-attachments/assets/9558a7db-7402-470e-acf0-299d3e36464e)
	![Image](https://github.com/user-attachments/assets/41420480-9f9b-4189-834e-7fe033e3a43e)

- Using your favorite model   
	You can download your favorite model [here](http://alice.org/pandagallery/).   
	I used these [model](http://alice.org/pandagallery/Vehicles/index.html).   
	Download the panda version and unzip the downloaded zip file.   
	Unzipping the zip file will create an egg file.   
	Here you specify the full path to the egg file, without the extension.   
	```self.model = self.loader.loadModel("path_to_model_file")```    
	![Image](https://github.com/user-attachments/assets/19088d8c-32c2-48f3-83a3-79adb364203a)
	![Image](https://github.com/user-attachments/assets/8b4a49e3-9954-4c6c-83aa-2d6636d9f629)
