

Requirements:
- We did all the requirements except for the second view (from the snake eyes), We succeded however in dividing the screen into two   parts which view the scene from above.
- The snake uses inverse kinamatics plus with the help of translation in order to reach a chosen object.
- With each level the number of objects we have to collect increases, and the spaces between them increases as well, each object   collected provides a number of points that can be spent in the shop for upgrades.



Additional points:
- We chose to implement the following topics:
	- Special snaky locomotion
	- Sound
	- Gravity and bouncing objects
		* Difficulties: at first objects were bouncing once and never coming back.
		  Solution: We had to make the number of frames irrelevant to movement and gravity, so we calculated the time between 		  frames, and instead used the time to determine how much movement and gravity we should add to each object.
	- Show score on the screen:
		* We tried to render the score on the screen, but we could not get imgui to work, so we chose to display the score on
			the window's top bar.
	- Your ideas:
		- Ability to increase snake speed by spending currency in the shop.
		- Ability to increase snake length by spending currency in the shop.
		- Having multiple lives (no enemies though... useless).
		- after finishing the objective of the current level, the snake must enter a cave to continue.
		- Calculting frames per second and showing it on the windows top bar.
		- A nice environment, built from objects we made.