"""import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from swift import Swift
import asyncio

# Fix for Windows event loop in Python 3.10+
asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

async def main():
    # Make and instance of the Swift simulator and open it
    env = Swift()
    env.launch(realtime=True)

    # Make a panda model and set its joint angles to the ready joint configuration
    panda = rtb.models.Panda()
    panda.q = panda.qr

    # Set a desired and effector pose an an offset from the current end-effector pose
    Tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.45)

    # Add the robot to the simulator
    env.add(panda)

    # Simulate the robot while it has not arrived at the goal
    arrived = False
    while not arrived:
        # Work out the required end-effector velocity to go towards the goal
        v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, 1)

        # Set the Panda's joint velocities
        panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v

        # Step the simulator by 50 milliseconds
        env.step(0.05)


asyncio.run(main())
"""

"""
import roboticstoolbox as rtb
from swift import Swift
import logging
logging.basicConfig(level=logging.DEBUG)
import nest_asyncio
nest_asyncio.apply()

# Set event loop policy
import asyncio
asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

# Test Swift
env = Swift()
env.launch(realtime=True, port=8080)

panda = rtb.models.Panda()
env.add(panda)
env.step()
"""
"""
import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np

env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.2, 0.2, 0.45)

arrived = False
env.add(panda)

dt = 0.05

while not arrived:

    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, 1)
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v
    env.step(dt)

# Uncomment to stop the browser tab from closing
# env.hold()
"""

import asyncio
import roboticstoolbox as rtb
from swift import Swift

async def main():
    env = Swift()
    await env.launch(realtime=True, port=8080)

    panda = rtb.models.Panda()
    env.add(panda)
    await asyncio.sleep(1)  # Keep the loop running for demo

    env.step()

if __name__ == "__main__":
    asyncio.run(main())


