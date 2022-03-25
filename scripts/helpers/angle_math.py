def diff(a: float, b: float) -> float:
    """
    Return the difference between two Euler angles.

    Both arguments should be in the range [0, 360). Returns a value in the range (-180, 180].

        Logic:
        If a <= 180 and a > b: a - b

    """
