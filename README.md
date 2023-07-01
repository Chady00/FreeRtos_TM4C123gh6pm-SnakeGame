# <span style="color: red;">FreeRtos_TM4C123gh6pm-SnakeGame</span>
## How the implementation works

### Overview
The code is a basic implementation of the Snake game in FreeRTOS, a real-time operating system. It utilizes FreeRTOS API to create tasks for printing and shifting the snake, as well as a mutex to handle game reset interruptions. The main tasks in the code are `PrintTask` and `ShiftSnake`. UART is used for communication, and various macro constants are defined for GPIO and UART configurations.

### Drivers used
- UART driver: Uart0 for sending, receiving, and displaying data on the serial terminal.
- GPIO driver to enable necessary pins.
- Systick Timer is used to trigger `vApplicationTickHook`.

### Tasks/Functions implemented

#### PrintTask (Higher Priority=3)
A task responsible for printing the updated frame of the Snake game to the terminal screen. It performs the following steps:
- Initializes the game frame by assigning each element of a 2-D array named "frame" with a space character.
- In an infinite loop, acquires a semaphore (xMutex) to gain exclusive access to shared resources.
- Updates the frame with different elements of the game, including the snake, enemies, and power-ups.
- Prints the updated frame to the terminal screen by calling the `uart0_putchar()` function for each element of the "frame" array.
- Prints the current score, high score, and time elapsed to the terminal screen using the `print()` function.
- Releases the semaphore (xMutex) after printing the information.
- Delays the task execution for a specific amount of time determined by the game speed.

#### ShiftSnake (Lower priority = 2)
A task that updates the state of the snake in the game. It performs the following steps:
- Moves the snake in one of the four directions (up, down, left, or right) based on the current direction of movement.
- Checks if the snake has hit any obstacles (its own tail or a wall) or food (normal point or power-up).
- If the snake hits a wall or its own tail, it resets the game.
- If the snake eats food, it increases the length of the snake and generates new food.
- Delays for a specified amount of time between each iteration to control the game speed.

#### ResetGame() function
A function that resets the game state and prepares it to start over. It performs the following tasks:
1. Takes control of a semaphore (xSemaphoreTake) with a maximum wait time (portMAX_DELAY).
2. Clears the screen (`print("\033[2J\033[1;1H")`) and resets the frame by filling it with spaces.
3. Resets the position of the snake to the center of the screen and sets its length to 3.
4. Adds a welcome message and game instructions to the frame.
5. Decreases the game speed if the player wins; otherwise, resets the game speed.
6. Adds a message "You win!" or "Game Over" depending on the score.
7. Displays the high score and the latest score on the screen.
8. Resets the score to 0 and waits for the start of the game.
9. Resets the time to 0 and clears the screen again.

#### Snake_power_up function
A function that updates a "special_pow" variable by incrementing it by 1 and taking the modulus with 5. It performs the following steps:
- Updates the location of a "power" object using a "do-while" loop.
- Clears the current position of the "power" object in the "frame" array.
- Generates a new random position for the "power" object by using the "rand()" function and taking the modulus with the frame width and length.
- Continues the loop until the new position of the "power" object does not coincide with the position of a snake body segment or a wall ("o" or "#").

### Faced Issues
1. Concurrency issues: Handling synchronization between tasks using semaphores or mutexes.
2. Designing the game logic: Defining the rules of the game, such as snake movement, collision detection, and increasing snake length when it eats food.
3. Implementing smooth movement of the snake: Ensuring the snake moves smoothly without flickering on the screen.
4. Keyboard input: Reading input from a keyboard in an RTOS environment and handling input events.
5. Debugging issues: Debugging concurrent tasks with non-deterministic execution order.
6. Scoring system: Keeping track of the player's score and displaying it on the screen.
7. Gameplay balancing: Adjusting the game speed and difficulty level.
