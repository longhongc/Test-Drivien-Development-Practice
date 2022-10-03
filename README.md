# Test-Driven Development Practice
This practice is to implement a simple PID controller in a test-driven manner.   
There are a total of two groups. 
One group (Pair A) has to plan the interface and the plain class structure in advance.   
Test function should also be implemented in this stage.   
The other group (Pair B) will based on the existing template and documents to try to implement
the PID controller. The implementation will have to pass all the pre-defined test cases. 

All the development is done under the techinique of pair-programming, which includes two
programmers to work together at one workstation. One, the driver, writes code while the other,
the navigator, reviews each line of code. 
![image](https://user-images.githubusercontent.com/28807825/193426966-4ee6885a-eb3c-4deb-9f61-5e047493e4b0.png)

## PID implementation
The PID implementation follows the equation from Wikipedia.  
Most of the coding notation also follows Wikipedia.   
https://en.wikipedia.org/wiki/PID_controller  
![image](https://user-images.githubusercontent.com/28807825/193662743-988255e1-6759-4ec5-970f-75169b941a22.png)

### The class UML diagram
![image](https://user-images.githubusercontent.com/28807825/193664860-3747f2e7-fddc-49e3-8be9-2b2fc8d4e747.png)

## Roles of Pair-Programming
||Pair A|Pair B|
|-----|-----|-----|
|*Driver*|Chang-Hong Chen|Pavan Mantripragada|
|*Navigator*|Abhinav Garg|Po-Yu Huang|

## Build
```
mkdir build && cd build
cmake ..
make -j
```

## Run
Run test
```
cd build/test
./cpp-test
```
