# FreeRTOS - Task Synchronization

Hi everyone,

This repository was created to share how I built a system with 4 independent tasks working with shared resources.

## Task Descriptions

- **Task 1:** Sends generic information via UART, and then sends packages to Task 2 and Task 4.
- **Task 2:** Checks the received message from the queue. If the message was sent to the correct receiver, it proceeds. Otherwise, it puts the received message back into the queue.
- **Task 3:** Wakes up after a semaphore release, which is dependent on a GPIO input. Then, it reads the ADC value, sends it via UART, and sends a package to Task 2.
- **Task 4:** Checks the received message from the queue. If the message was sent to the correct receiver, it proceeds. Otherwise, it puts the received message back into the queue.

## Overview

In general, the system does not have a very significant usual purpose, but it proves the importance of task synchronization through the use of semaphores and mutexes.

And sorry for any English typos, I'm currently studying to improve it as well.
