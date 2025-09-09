import re
from PySide6.QtCore import QObject
from PySide6.QtWidgets import QPushButton
from common_msgs.msg import Recipe

def parse_height_depth(text: str):
    """
    Extracts height and depth from a button text like:
    'C1R1\nH=944.0, D=600.0'
    Returns (height, depth) as floats, or (None, None) if not found.
    """
    match = re.search(r"H\s*=\s*([\d.]+).*D\s*=\s*([\d.]+)", text)
    if match:
        height = float(match.group(1))
        depth = float(match.group(2))
        return height, depth
    return None, None

class CabinetsController(QObject):
    def __init__(self, ui, ros_node):
        super().__init__()

        self.ui = ui
        self.ros_node = ros_node

        self._recipe_height = None
        self._recipe_depth = None
        self._recipe_mode = None
        self._last_button = None  # optional: store which button was clicked

        self.ui.PickRecipeButton.toggled.connect(self.on_recipe_mode)
        self.ui.AssemblyRecipeButton.toggled.connect(self.on_recipe_mode)

        self.ui.SavePickCabinet.clicked.connect(self.on_save_cabinet)
        self.ui.SaveAssemblyCabinet.clicked.connect(self.on_save_cabinet)

        self.ui.CancelPickCabinet.clicked.connect(self.on_cancel_cabinet)
        self.ui.CancelAssemblyCabinet.clicked.connect(self.on_cancel_cabinet)

        # Connect all buttons in Pick and Assembly grids
        self._connect_buttons(self.ui.GridLayoutPick)
        self._connect_buttons(self.ui.GridLayoutAssembly)

    def _connect_buttons(self, grid_layout):
        """Helper to connect every QPushButton in a grid layout."""
        for row in range(grid_layout.rowCount()):
            for col in range(grid_layout.columnCount()):
                item = grid_layout.itemAtPosition(row, col)
                if not item:
                    continue
                widget = item.widget()
                if isinstance(widget, QPushButton):
                    print("Connected button:", widget.objectName())
                    widget.clicked.connect(self.on_recipe_button_clicked)

    def on_recipe_mode(self, checked: bool):
        """
        Hooked to toggled() of mode buttons.
        Uses sender() to figure out which button fired.
        Only sets mode when the sender is checked.
        """
        btn = self.sender()
        if not btn or not checked:
            return

        name = getattr(btn, "objectName", lambda: "")()
        # Map button identity → mode string expected by Recipe.msg
        if name == "PickRecipeButton":
            self._recipe_mode = "pick"

            self.ui.MainPageAutoAndManualStackedWidget.setCurrentIndex(1)
        elif "AssemblyRecipeButton" in name:  # supports "AssemblyButton" or "AssemblyRecipeButton"
            self._recipe_mode = "assembly"

            self.ui.MainPageAutoAndManualStackedWidget.setCurrentIndex(2)
        else:
            # If some other button wired by accident, ignore
            return

        print(f"[Recipe] Mode set → {self._recipe_mode}")

    def on_recipe_button_clicked(self):
        btn = self.sender()
        if not isinstance(btn, QPushButton):
            return

        btn_text = btn.text()
        self._recipe_height, self._recipe_depth = parse_height_depth(btn_text)
        self._last_button = btn  # optional, if you need to know which one

        print(f"[UI] Selected {btn.objectName()} → Height={self._recipe_height}, Depth={self._recipe_depth}")
    


    def on_save_cabinet(self):
        self.send_recipe()
        self.ui.MainPageAutoAndManualStackedWidget.setCurrentIndex(0)

    def on_cancel_cabinet(self):
        self.ui.MainPageAutoAndManualStackedWidget.setCurrentIndex(0)

    
    def send_recipe(self) -> bool:
        """
        Collects current recipe mode + height, validates, and publishes /recipe.
        Returns True if published, False otherwise.
        """
        # Prefer the cached value; if missing, try to parse current text box contents
        # if self._recipe_height is None:
        #     self.on_height_recipe_input_changed(self.ui.HeightRecipeInput.text())
    
        # if self._recipe_depth is None:
        #     self.on_depth_recipe_input_changed(self.ui.DepthRecipeInput.text())

        # Validate
        if not self._recipe_mode:
            print("[Recipe] ERROR: Mode not selected (pick/assembly).")
            return False

        if self._recipe_height is None:
            print("[Recipe] ERROR: Height is empty or invalid.")
            return False
        
        if self._recipe_depth is None:
            print("[Recipe] ERROR: Depth is empty or invalid.")
            return False

        # Build and publish
        msg = Recipe()
        msg.mode = self._recipe_mode
        msg.height = float(self._recipe_height)
        msg.depth = float(self._recipe_depth)

        print(f"[DEBUG] Publishing Recipe → mode:{msg.mode} height:{msg.height} depth:{msg.depth}")
        self.ros_node.recipe_publisher.publish(msg)
        print("[UI] Sent Recipe to /recipe")

        return True
