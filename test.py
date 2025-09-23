import numpy as np
from dataclasses import dataclass

class InventoryItem:
    '''Class for keeping track of an item in inventory.'''
    name: str
    unit_price: float
    quantity_on_hand: int = 0

    def __init__(
            self, 
            name: str, 
            unit_price: float,
            quantity_on_hand: int = 0
        ) -> None:
        self.name = name
        self.unit_price = unit_price
        self.quantity_on_hand = quantity_on_hand

    def total_cost(self) -> float:
        return self.unit_price * self.quantity_on_hand
    
    def __repr__(self) -> str:
        return (
            'InventoryItem('
            f'name={self.name!r}, unit_price={self.unit_price!r}, '
            f'quantity_on_hand={self.quantity_on_hand!r})'
        )

    def __hash__(self) -> int:
        return hash((self.name, self.unit_price, self.quantity_on_hand))

    def __eq__(self, other) -> bool:
        if not isinstance(other, InventoryItem):
            return NotImplemented
        return (
            (self.name, self.unit_price, self.quantity_on_hand) == 
            (other.name, other.unit_price, other.quantity_on_hand))
    
@dataclass
class SimulationData:
    '''Class for keeping track of an item in inventory.'''
    name: str

    simulation_step: np.ndarray
    position_history: np.ndarray
    velocity_history: np.ndarray

    max_speed_history: np.ndarray
    min_speed_history: np.ndarray
    average_speed_history: np.ndarray
    range_history: np.ndarray

if __name__ == '__main__':
    array = SimulationData([], [3, 2, 1], [], [], [], [], [], [])
    array.simulation_step.sort()
    print(array)
    print(type(array))