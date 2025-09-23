import numpy as np

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
    
if __name__ == '__main__':
    array = InventoryItem("Game", 60, 5)
    print(array)
