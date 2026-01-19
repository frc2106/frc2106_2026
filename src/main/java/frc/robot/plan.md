Robot State Machine:

INTAKE (fold intake out, spin it, collect balls)
IDLE (fold intake in, dynamic shooter wheel rpm)
EJECT (shoot balls out of intake)

SHOOTING_WAITING (spinning and getting correct angle + speed)
SHOOTING_READY (robot is ready to shoot)
SHOOTING_ACTIVE (robot is activly shooting) (shooter spins wheels to expected rpm, intake pushes balls into it, robot auto aligns as well)


Intake:
    Wheel Motor X60
    Expansion Motor X60

    
Shooter:
    Four motors, all same PID, connected. X60s
    Hood X60

Indexer:
    Kicker Motor X60
    Conveyor Motor X60
    

Drive:
    8 Kraken X60s
    
    
Vision:
    Quad cameras (ideal)
    Dual Orange Pis
    

RobotStatus
    
    

- By default robot is is in IDLE, pressing B idles WHOLE robot
- When we hold down intake, robot enters INTAKE
- When we hold down shoot, robot starts at SHOOTING_WAITING, and adjust accordingly.
