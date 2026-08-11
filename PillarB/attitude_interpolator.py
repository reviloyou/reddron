"""
attitude_interpolator.py

Time indexd store of attitude samples. Given a timestamp in the
flight controller`s clock (time_boot_ms), returns the interpolated
attitude quaternion at the instant, or None if the query cannot be 
aquired in a correct way, it might be (too old, too new, or bridging to large a gap)

Clock-offset estimation (ESP32 millis() vs FC time_boot_ms) is not done/included here.
This is done by collecting the offset in actual operation.
"""

import numpy as np

NLERP_FALLBACK_DOT = 0.9995 #This is beacuse SLERP gets unrealiable beacuse of sin(theta) division over this

def _normalize(q):
    n = np.linalg.norm(q)
    return q / n

def _slerp(q0, q1, frac):
    q0 = _normalize(np.asarray(q0, dtype=np.float64))
    q1 = np.asarray(q1, dtype=np.float64)

    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    q1 = _normalize(q1)

    if dot >  NLERP_FALLBACK_DOT:
        result = (1.0 - frac) * q0 + frac * q1
        return _normalize(result)

    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta = np.sin(theta)
    a = np.sin((1.0 - frac) * theta) / sin_theta
    b = np.sin(frac * theta) / sin_theta
    return _normalize(a * q0 + b * q1)

class AttitudeBuffer:
    def __init__(self, capacity=64, max_gap_ms=200):
        if capacity < 2:
            raise ValueError("capacity must be at least 2")
        self._capacity = capacity
        self._max_gap_ms = max_gap_ms

        self._t = np.zeros(capacity, dtype=np.int64)
        self._q = np.zeros((capacity, 4), dtype=np.float64)
        self._next = 0
        self._count = 0

        self.rejected_too_old = 0
        self.rejected_too_new = 0
        self.rejected_gap = 0
        self.queries_served = 0

    def _phys(self, logical_i):
        return (self._next - self._count + logical_i) % self._capacity

    def add(self, t_ms, q):
        q = np.asarray(q, dtype=np.float64)
        if q.shape !=(4,):
            raise ValueError(f"q must have shaoe (4,), got {q.shape}")

        if self._count > 0:
            last_idx = (self._next -1) % self._capacity
            if t_ms <= self._t[last_idx]:
                return False

        self._t[self._next] = t_ms
        self._q[self._next] = q
        self._next = (self._next +1) % self._capacity
        self._count = min(self._count + 1, self._capacity)
        return True

    def at(self, t_ms):
        if self._count < 2:
            self.rejected_too_old += 1
            return None

        t_oldest = self._t[self._phys(0)]
        t_newest = self._t[self._phys(self._count -1)]

        if t_ms < t_oldest:
            self.rejected_too_old += 1
            return None
        if t_ms > t_newest:
            self.rejected_too_new += 1
            return None

        i1 = self._count - 1
        while i1 > 0 and self._t[self._phys(i1 - 1)] > t_ms:
            i1 -= 1
        i0 = i1 -1

        t0 = self._t[self._phys(i0)]
        t1 = self._t[self._phys(i1)]
        q0 = self._q[self._phys(i0)]
        q1 = self._q[self._phys(i1)]

        if t_ms == t0:
            self.queries_served += 1
            return tuple(_normalize(q0))
        if t_ms == t1:
            self.queries_served += 1
            return tuple(_normalize(q1))

        if (t1 - t0) > self._max_gap_ms:
            self.rejected_gap += 1
            return None

        frac = (t_ms - t0) / (t1 - t0)
        result = _slerp(q0, q1, frac)
        self.queries_served += 1
        return tuple(result)