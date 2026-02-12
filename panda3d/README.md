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
	![Image](https://github.com/user-attachments/assets/cb9f1408-b372-4e55-84f6-1191f068cd1a)

- using biplain mode   
	![Image](https://github.com/user-attachments/assets/dea38ac9-726a-441d-8861-ce71a415f1e4)

- using 707 mode   
	![Image](https://github.com/user-attachments/assets/d0984d71-ac14-4579-8915-19d0e55e6659)

- using fa18 mode   
	![Image](https://github.com/user-attachments/assets/c8a53720-0ddc-4b2b-b729-6cfd8fbbe2ea)

- Move the camera forward and backward   
	Hold down the right button and move the mouse.   

- Move the camera up, down, left and right   
	Hold down the left button and move the mouse.   
	![Image](https://github.com/user-attachments/assets/3e3c6c96-ecad-41f9-a9dd-e76caacc6f0d)
	![Image](https://github.com/user-attachments/assets/68f521bd-f37a-439d-af5f-ced5cf1fadae)

- Using your favorite model   
	You can download your favorite model [here](http://alice.org/pandagallery/).   
	I used these [model](http://alice.org/pandagallery/Vehicles/index.html).   
	Download the panda version and unzip the downloaded zip file.   
	Unzipping the zip file will create an model file and texture file.   
	Here you specify the full path to the model file and texture file.   
	```
	self.model = self.loader.loadModel("path_to_model_file")
	tex = loader.loadTexture("path_to_texture_file")
	self.model.setTexture(tex, 1)
	```    
	![Image](https://github.com/user-attachments/assets/46c9d3bb-25f9-46da-a56a-ef8b3de212cf)
