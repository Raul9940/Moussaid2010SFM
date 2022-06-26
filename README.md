# Moussaid2010SFM
Library with codes simulate the Moussaid et al. extension on the Social Force Model introduced in 2010.

This model is based in the original Social Force Model introduced in 1995 by D. Helbing and P. Molnár [1], with the addition of pedestrians walking in groups. The model is presented in [2].

For now, there are two different configurations of the walkable area that can be run:

-[Empty corridor](<Codes/empty_corridor.py>): People move in an empty corridor only limited by two horizontal walls in the y axis.

-[Empty corridor with variable size in the middle](<Codes/middle_aperture.py>): Same configuration as in the previous case, but now two walls are placed in the central part of the corridor, so width of it there is reduced.

Simulations are programmed in a Runge-Kutta 4 scheme with periodic boundary conditions to keep a continuous flow of people.

References:

[1] D. Helbing and P. Molnár. Social force model for pedestrians dynamics. Phys. Rev. E, 51:4282-4286. May 1995.

[2] M. Moussaïd, N. Perozo, S. Gaminer, D. Helbing and G. Theraulaz. The walking behaviour of pedestrian social groups and impact on crowd dynamics. PloS one 5(4). 2010.
