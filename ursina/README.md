# View Euler angles using library
You can view Euler angles using [this](https://www.ursinaengine.org/) library.   
It works as a UDP display server.   

```
+-------------+          +-------------+          +-------------+
|             |          |             |          |             |
|     IMU     |--(ic2)-->|    ESP32    |--(UDP)-->|   main.py   |
|             |          |             |          |             |
+-------------+          +-------------+          +-------------+
```

```
$ python3 -m pip install ursina
$ python3 main.py --help
usage: main.py [-h] [--texture TEXTURE]

options:
  -h, --help         show this help message and exit
  --texture TEXTURE  texture
```

# Texture
We can use these textures.   
```
noise	
grass	
vignette	
arrow_right	
test_tileset	
tilemap_test_level	
shore	
file_icon	
sky_sunset	
radial_gradient	
circle	
perlin_noise	
brick	
grass_tintable	
circle_outlined	
ursina_logo	
arrow_down	
cog	
vertical_gradient	
white_cube	
horizontal_gradient	
folder	
rainbow	
heightmap_1	
sky_default
```

- using brick texture  (default texture)   
	<img width="720" height="480" alt="Image" src="https://github.com/user-attachments/assets/c6526beb-fa37-4f78-9919-a569d3c6165c" />

- using white_cube texture   
	<img width="720" height="480" alt="Image" src="https://github.com/user-attachments/assets/824f025d-8428-4c66-84cb-68ea9f4ec3f2" />

- using sky_sunset texture   
	<img width="720" height="480" alt="Image" src="https://github.com/user-attachments/assets/d4724ea8-03a9-4c57-a493-a16e5260779d" />

- using ursina_logo texture   
	<img width="720" height="480" alt="Image" src="https://github.com/user-attachments/assets/2d6d5972-fcf2-48e2-a539-b40b1d157809" />

