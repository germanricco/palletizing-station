"""
Módulo para el Gestor de Recetas de Palletizado.
Permite cargar una única trayectoria o armar una secuencia (receta).
"""
import os
from pathlib import Path
from typing import List, Tuple, Optional

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QListWidget, QListWidgetItem, QAbstractItemView, QWidget
)
from PySide6.QtCore import Qt, Signal
from app.core.constants import AppConfig

class RecipeListWidget(QListWidget):
    """
    Subclase de QListWidget para capturar el final exacto de un 
    evento de Drag & Drop y notificar a la interfaz para reordenar.
    """
    order_changed = Signal()

    def dropEvent(self, event):
        # 1. Dejamos que Qt haga su magia y mueva el ítem a su nueva posición
        super().dropEvent(event)
        
        # 2. Emitimos la señal de que el movimiento ha concluido
        self.order_changed.emit()

class RecipeBuilderDialog(QDialog):
    """
    Diálogo de doble columna para selección y armado de recetas.
    Retorna (tipo_carga, datos). tipo_carga puede ser 'SINGLE' o 'RECIPE'.
    """
    def __init__(self, trajectories_dir: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Gestor de Trayectorias y Recetas")
        self.setFixedSize(650, 450)
        self.setModal(True)
        self.trajectories_dir = trajectories_dir
        
        self.result_type: Optional[str] = None
        self.result_data: List[str] = []

        self.setStyleSheet(
            f"QDialog {{ background-color: {AppConfig.COLORS.SURFACE}; border: 1px solid {AppConfig.COLORS.BORDERS}; }}"
            f"QLabel {{ color: {AppConfig.COLORS.TEXT_HIGH}; font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; font-weight: bold; font-size: 12px; }}"
            f"QListWidget {{ background-color: #0A0A0A; color: {AppConfig.COLORS.TEXT_HIGH}; border: 1px solid {AppConfig.COLORS.BORDERS}; border-radius: 4px; padding: 4px; font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA}; }}"
            f"QListWidget::item:selected {{ background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; color: #121212; }}"
        )
        
        self._setup_ui()
        self._load_available_files()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(16)

        # --- ZONA SUPERIOR: Listas ---
        lists_layout = QHBoxLayout()
        
        # Columna Izquierda: Archivos Disponibles
        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("TRAYECTORIAS DISPONIBLES"))
        self.list_available = QListWidget()
        self.list_available.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        left_layout.addWidget(self.list_available)
        
        # Botones Centrales (Transferencia)
        center_layout = QVBoxLayout()
        center_layout.addStretch()
        
        btn_style = f"QPushButton {{ background-color: #2A2A2A; color: {AppConfig.COLORS.TEXT_HIGH}; border-radius: 4px; font-weight: bold; padding: 8px; }}"
        
        self.btn_add = QPushButton("▶ AÑADIR")
        self.btn_add.setStyleSheet(btn_style)
        self.btn_add.clicked.connect(self._add_to_recipe)
        
        self.btn_remove = QPushButton("◀ QUITAR")
        self.btn_remove.setStyleSheet(btn_style)
        self.btn_remove.clicked.connect(self._remove_from_recipe)
        
        center_layout.addWidget(self.btn_add)
        center_layout.addWidget(self.btn_remove)
        center_layout.addStretch()

        # Columna Derecha: Receta Armada
        right_layout = QVBoxLayout()
        right_layout.addWidget(QLabel("RECETA DE PALLETIZADO (ORDEN)"))

        self.list_recipe = RecipeListWidget()
        self.list_recipe.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.list_recipe.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        
        self.list_recipe.order_changed.connect(self._update_recipe_indices)
        
        right_layout.addWidget(self.list_recipe)

        lists_layout.addLayout(left_layout, stretch=2)
        lists_layout.addLayout(center_layout, stretch=1)
        lists_layout.addLayout(right_layout, stretch=2)
        
        main_layout.addLayout(lists_layout)

        # --- ZONA INFERIOR: Acciones ---
        action_layout = QHBoxLayout()
        
        self.btn_cancel = QPushButton("CANCELAR")
        self.btn_cancel.setStyleSheet(btn_style)
        self.btn_cancel.clicked.connect(self.reject)

        self.btn_load_single = QPushButton("CARGAR ÚNICA (SELECCIÓN)")
        self.btn_load_single.setStyleSheet(f"QPushButton {{ background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; color: #121212; border-radius: 4px; font-weight: bold; padding: 8px 16px; }}")
        self.btn_load_single.clicked.connect(self._submit_single)

        self.btn_load_recipe = QPushButton("CARGAR RECETA (LISTA)")
        self.btn_load_recipe.setStyleSheet(f"QPushButton {{ background-color: {AppConfig.COLORS.SEMANTIC_AUTO}; color: #121212; border-radius: 4px; font-weight: bold; padding: 8px 16px; }}")
        self.btn_load_recipe.clicked.connect(self._submit_recipe)

        action_layout.addWidget(self.btn_cancel)
        action_layout.addStretch()
        action_layout.addWidget(self.btn_load_single)
        action_layout.addWidget(self.btn_load_recipe)
        
        main_layout.addLayout(action_layout)

    def _load_available_files(self):
        if not os.path.exists(self.trajectories_dir):
            os.makedirs(self.trajectories_dir)
        for f in os.listdir(self.trajectories_dir):
            if f.endswith(".npy"):
                self.list_available.addItem(Path(f).stem)

    def _add_to_recipe(self):
        """Añade los elementos seleccionados y actualiza los índices."""
        for item in self.list_available.selectedItems():
            pure_text = item.text()
            
            # Creamos un nuevo ítem en lugar de copiarlo
            new_item = QListWidgetItem()
            # Guardamos el nombre real del archivo de forma oculta
            new_item.setData(Qt.ItemDataRole.UserRole, pure_text) 
            
            self.list_recipe.addItem(new_item)
            
        self._update_recipe_indices()

    def _remove_from_recipe(self):
        """Elimina los elementos seleccionados y actualiza los índices."""
        for item in self.list_recipe.selectedItems():
            self.list_recipe.takeItem(self.list_recipe.row(item))
            
        self._update_recipe_indices()

    def _update_recipe_indices(self):
        """
        Itera sobre toda la lista de recetas y actualiza el texto visible
        para asegurar que la numeración siempre sea secuencial (1, 2, 3...).
        """
        for i in range(self.list_recipe.count()):
            item = self.list_recipe.item(i)
            # Recuperamos el nombre real que guardamos en UserRole
            pure_text = item.data(Qt.ItemDataRole.UserRole)
            if pure_text:
                # Modificamos solo lo que el usuario ve
                item.setText(f"{i + 1}. {pure_text}")

    def _submit_single(self):
        selected = self.list_available.selectedItems()
        if selected:
            self.result_type = 'SINGLE'
            self.result_data = [selected[0].text()]
            self.accept()

    def _submit_recipe(self):
        """Envía la receta final al orquestador usando los nombres reales de los archivos."""
        if self.list_recipe.count() > 0:
            self.result_type = 'RECIPE'
            
            # Extraemos la información oculta (UserRole) para no enviar el "1. " al STM32
            self.result_data = [
                self.list_recipe.item(i).data(Qt.ItemDataRole.UserRole) 
                for i in range(self.list_recipe.count())
            ]
            self.accept()

    def get_results(self) -> Tuple[str, List[str]]:
        return self.result_type, self.result_data