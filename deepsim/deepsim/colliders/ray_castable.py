import abc
from typing import Union

from deepsim.colliders.hit import Hit
from deepsim.core.ray import Ray


# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class RayCastable(ABC):
    """
    Ray Castable interface.
    """
    @abc.abstractmethod
    def raycast(self, ray: Ray) -> Union[Hit, None]:
        """
        Returns the distance along the ray, where it intersects the ray-castable object.
        - If there is no intersection then returns None.

        Args:
            ray (Ray): ray to test intersection.

        Returns:
            Union[Hit, None]: Hit object if intersects. Otherwise, None.
        """
        raise NotImplementedError("Subclass must implement raycast!")
