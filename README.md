# 4D_ULTRA_ENGINE
An engine in C that converts .txt files to objects in 3 dimensions, with a 4th one adding to the craziness. Compile it and type text_name.txt on the terminal. Requires GTK library. Have fun!  
Controls are:  
  WASD for moving, usual  
  mouse for lookin around  
  L to lock the screen  
try something like:  
```
gcc -o 4d_ultra_engine 4d_ultra_engine.c  `pkg-config --cflags --libs gtk+-3.0`
./4d_ultra_engine 5-cell.txt
```


	
