"""Small 2-D helpers: axis-aligned rectangles, rotation, polygons."""

from __future__ import annotations

import math
from dataclasses import dataclass

EPS = 1e-6

Point = tuple[float, float]


def dm(value: float) -> int:
    """Metres to integer decimetres (the grid on which floor plans are laid out)."""
    return round(value * 10)


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def w(self) -> float:
        return self.x1 - self.x0

    @property
    def h(self) -> float:
        return self.y1 - self.y0

    @property
    def area(self) -> float:
        return self.w * self.h

    @property
    def center(self) -> Point:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)

    def inset(self, d: float) -> Rect:
        return Rect(self.x0 + d, self.y0 + d, self.x1 - d, self.y1 - d)

    def intersects(self, o: Rect, eps: float = EPS) -> bool:
        """True when the interiors overlap (touching edges do not count)."""
        return (
            self.x0 < o.x1 - eps
            and o.x0 < self.x1 - eps
            and self.y0 < o.y1 - eps
            and o.y0 < self.y1 - eps
        )

    def contains(self, o: Rect, eps: float = EPS) -> bool:
        return (
            o.x0 >= self.x0 - eps
            and o.y0 >= self.y0 - eps
            and o.x1 <= self.x1 + eps
            and o.y1 <= self.y1 + eps
        )

    def contains_point(self, p: Point, eps: float = EPS) -> bool:
        return self.x0 - eps <= p[0] <= self.x1 + eps and self.y0 - eps <= p[1] <= self.y1 + eps

    def distance_to_point(self, p: Point) -> float:
        dx = max(self.x0 - p[0], 0.0, p[0] - self.x1)
        dy = max(self.y0 - p[1], 0.0, p[1] - self.y1)
        return math.hypot(dx, dy)

    def corners(self) -> list[list[float]]:
        """Counter-clockwise corners as [x, y] lists."""
        return [[self.x0, self.y0], [self.x1, self.y0], [self.x1, self.y1], [self.x0, self.y1]]

    @staticmethod
    def from_points(points: list[list[float]] | list[Point]) -> Rect:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return Rect(min(xs), min(ys), max(xs), max(ys))


def rotate(p: Point, yaw: float) -> Point:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def polygon_area(poly: list[list[float]] | list[Point]) -> float:
    total = 0.0
    for i, (x0, y0) in enumerate(poly):
        x1, y1 = poly[(i + 1) % len(poly)]
        total += x0 * y1 - x1 * y0
    return abs(total) / 2


def point_in_polygon(p: Point, poly: list[list[float]] | list[Point]) -> bool:
    x, y = p
    inside = False
    for i, (x0, y0) in enumerate(poly):
        x1, y1 = poly[(i + 1) % len(poly)]
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            inside = not inside
    return inside


def rect_in_polygon(r: Rect, poly: list[list[float]] | list[Point], n: int = 5) -> bool:
    """Sampled test that a rectangle lies inside a (rectilinear) polygon."""
    e = 1e-4
    for i in range(n):
        for j in range(n):
            x = r.x0 + e + (r.w - 2 * e) * i / (n - 1)
            y = r.y0 + e + (r.h - 2 * e) * j / (n - 1)
            if not point_in_polygon((x, y), poly):
                return False
    return True
