from core.domain.result import Result

from .sdk import BasketVisionClient


class BasketVisionMixin:
    """Basket vision methods exposed from LejuWheeledArmHardware."""

    def _get_basket_vision_client(self) -> BasketVisionClient:
        if getattr(self, "basket_vision", None) is None:
            config = self.config.get("basket_vision", {}) if hasattr(self, "config") else {}
            self.basket_vision = BasketVisionClient(config=config)
        return self.basket_vision

    def wait_basket_vision_ready(self) -> Result:
        """Wait until basket vision ROS services are available."""
        return self._get_basket_vision_client().wait_until_ready()

    def infer_basket_pose(self) -> Result:
        """Infer visible basket poses."""
        return self._get_basket_vision_client().infer_basket_pose()

    def infer_top_basket(self) -> Result:
        """Infer the top/target basket pose."""
        return self._get_basket_vision_client().infer_top_basket()