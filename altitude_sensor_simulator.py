import random
import time


class AltitudeSimulator:
    def __init__(
        self,
        min_alt=500.0, # feet
        max_alt=10_000.0, # feet
        max_fpm=1500.0, # max climb/descent rate
        max_accel_fps2=5.0 # how "aggressive" altitude changes feel
    ):
        self.min_alt = min_alt
        self.max_alt = max_alt

        self.altitude = random.uniform(min_alt, max_alt)
        self.vert_speed = 0.0 # ft/sec
        self.max_speed = max_fpm / 60 # ft/sec
        self.max_accel = max_accel_fps2

        self._last_time = time.time()

    def update(self):
        """Advance the simulation and return the current altitude."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if dt <= 0:
            return self.altitude

        # Randomly perturb vertical speed (acceleration)
        accel = random.uniform(-self.max_accel, self.max_accel)
        self.vert_speed += accel * dt

        # Limit climb/descent rate
        self.vert_speed = max(
            -self.max_speed,
            min(self.vert_speed, self.max_speed)
        )

        # Integrate altitude
        self.altitude += self.vert_speed * dt

        # Bounce off altitude bounds
        if self.altitude <= self.min_alt:
            self.altitude = self.min_alt
            self.vert_speed = abs(self.vert_speed)

        elif self.altitude >= self.max_alt:
            self.altitude = self.max_alt
            self.vert_speed = -abs(self.vert_speed)

        return self.altitude
