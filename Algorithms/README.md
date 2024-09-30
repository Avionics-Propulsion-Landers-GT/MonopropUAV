**Active Tasks**

1. TVC Algorithm - Ben Mamut, Sanchir Justin Erdenbat, Nathan Kim, Kyle Law, Richard Ortiz
2. Dynamics Modeling - Yash Buddhdeo, Sai Lalith Kanumuri, Mahir Daiyan Ashraf, Saleem Salick
3. Path Planning - Jaehun Baek
4. RCS Modeling

**Algorithms/TVC/TVCDynamics_Simulator.m**  
This simulation takes in a desired orientation vector and calculates the required pitch and yaw rotations required to make that happen. 
It then outputs the pulse width duration that the servos will use to rotate the TVC accordingly. 
Additionally, it recalculates the desired normal vector and compares it against the inputted vector for correctness.
