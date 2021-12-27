# PDMCoffeeBringer

Instructions for running the rrt_star code:
Just compile using the CMakeList.txt file in the directory "rrt_star".
Beware however in line 10 of this file you will have to change the variable "PATH_TO_FINDSFML" to **your** filepath of "FindSFML.cmake". Of course you will also need to have installed both the SFML and eigen libraries.

When running the code **be careful with closing the window, because that will terminate the algorithm**, after which results will be saved in "optimal_path.txt" in the directory "rrt_star".
