screenshot of the circuit
![Screenshot (194)](https://github.com/Ridhima-Sharma-2005/Awesome_Robotics_Club_Ridhima-Sharma_230854/assets/147236518/d00d2f4a-871d-455f-ab62-41dd7e185bf3)


link of the tinkercad file
https://www.tinkercad.com/things/5nuZPh8Mb7n-flappy-bird
https://www.tinkercad.com/things/5nuZPh8Mb7n-flappy-bird/editel?returnTo=%2Fthings%2F5nuZPh8Mb7n-flappy-bird
any of the above links will work fine


explaination of the code
The code starts with incuding a LiquidCrystal library required to operate the lcd
then some of the variables are defined with used  pins of arduino according to their respective functions.
for example, where all the pins of lcd like rs,e,db4,db5,db6,db7 and led are connected to arduino board . here it is 12,11,7,5,4,3 digital pins. 
the potentiometer used to control the position of the flappy bird is connected to the analog pins of arduino board. 
the given 2 obstacle strings are stored in two variables typea and typeaa and the positon of occurence of '*' is stored in other two arrays namely typeA and typeAA
In the setup function, the code for initializing the start of the game is given by reading the value of potentiometer. when the value of potentiometer is more than half of its max value , the game will start and our flappy bird will be displayed in the form of a 'O'.
now the code that will be running in the loop function will read the potentiometer reading after every iteration and will change the positino of  'O' according . i.e. when it is half more than max the 'O' will be at [0,0] otherwise at [0,1]
inside the loop function we have another loop to print the obstacle array. a varaible pos is uesd here to keep a check on the 'O' in lcd (0 or 1).
the for loop is used to print the strings in the top and bottom line .when the bird is in top the top string will be printed from 1 value ahead (i.e.i+1) in the top line and when it is in the bottom line , the string in the bottom line will be printed  value ahead
to check the collision of the bird with the obstacles , we require the index of the bird and the obstacles . for this we use the list initially defined (containing the positions of obstacles). when the index of the bird is one among the array initialized the game ends .
when the entire 2 arrays given to us are printed and bird passes all of the obstacles then the game ends and the bird wins.
