import sys
import os
import shutil
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QFileDialog,
    QVBoxLayout, QHBoxLayout, QListWidget, QMessageBox, QCheckBox,
    QTextEdit, QDialog, QColorDialog, QDialogButtonBox
)
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt5.QtCore import Qt, QPointF
import random


COLORS = [Qt.red, Qt.blue, Qt.green, Qt.cyan, Qt.yellow]

class SettingsDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("설정")
        self.resize(300, 200)
        layout = QVBoxLayout()
        self.auto_delete_checkbox = QCheckBox("다음 시 자동 저장 및 원본 삭제")
        layout.addWidget(self.auto_delete_checkbox)
        self.color_buttons = []
        for i in range(2):
            btn = QPushButton(f"차선 {i+1} 색상 변경")
            btn.clicked.connect(lambda _, idx=i: self.change_color(idx))
            layout.addWidget(btn)
            self.color_buttons.append(btn)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)
        self.setLayout(layout)

    def change_color(self, idx):
        color = QColorDialog.getColor()
        if color.isValid():
            COLORS[idx] = color

class LabelTool(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CurveLane Labeling Tool")
        self.resize(1200, 800)

        self.image_paths = []
        self.current_index = 0
        self.lanes = [[]]
        self.current_lane_idx = 0
        self.undo_stack = [[]]
        self.redo_stack = [[]]
        self.auto_track_enabled = False
        self.last_auto_point = None
        self.auto_delete = False

        self.saved_dir = "Processed"
        os.makedirs(self.saved_dir, exist_ok=True)
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()
        side_layout = QVBoxLayout()

        self.list_widget = QListWidget()
        side_layout.addWidget(self.list_widget)

        btn_layout = QHBoxLayout()
        self.btn_load = QPushButton("폴더 불러오기")
        self.btn_prev = QPushButton("이전")
        self.btn_next = QPushButton("다음")
        self.btn_save = QPushButton("저장")
        self.btn_reset = QPushButton("초기화")
        self.btn_switch_lane = QPushButton("다음 차선")
        self.btn_settings = QPushButton("설정")
        self.btn_make = QPushButton("데이터셋 제작")

        for btn in [
            self.btn_load, self.btn_prev, self.btn_next,
            self.btn_save, self.btn_reset, self.btn_switch_lane,
            self.btn_settings, self.btn_make
        ]:
            btn_layout.addWidget(btn)

        side_layout.addLayout(btn_layout)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        side_layout.addWidget(self.log_box)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMouseTracking(True)
        self.setMouseTracking(True)

        main_layout.addLayout(side_layout, 1)
        main_layout.addWidget(self.image_label, 3)

        self.setLayout(main_layout)

        self.btn_load.clicked.connect(self.load_images)
        self.btn_prev.clicked.connect(self.show_prev)
        self.btn_next.clicked.connect(self.show_next)
        self.btn_save.clicked.connect(self.save_labels)
        self.btn_reset.clicked.connect(self.reset_points)
        self.btn_switch_lane.clicked.connect(self.switch_lane)
        self.btn_settings.clicked.connect(self.show_settings)
        self.image_label.mousePressEvent = self.image_clicked
        self.btn_make.clicked.connect(self.make_dataset)

    def make_dataset(self):
        self.train_dir = "Curvelanes/train/"
        self.train_img_dir = "Curvelanes/train/images"
        self.train_label_dir = "Curvelanes/train/labels"
        self.valid_dir = "Curvelanes/valid/"
        self.valid_img_dir = "Curvelanes/valid/images"
        self.valid_label_dir = "Curvelanes/valid/labels"
        self.test_img_dir = "Curvelanes/test/images"
        os.makedirs(self.train_img_dir, exist_ok=True)
        os.makedirs(self.train_label_dir, exist_ok=True)
        os.makedirs(self.valid_img_dir, exist_ok=True)
        os.makedirs(self.valid_label_dir, exist_ok=True)
        os.makedirs(self.test_img_dir, exist_ok=True)

        unsort_list = os.listdir(self.saved_dir)
        unsort_list = list(filter(lambda f: f.endswith(".lines.json"), unsort_list))
        new_list = []

        for i in unsort_list:
            unsort_list.remove(i)
            new_list.append(i.replace(".lines.json", ""))
            self.log(i.replace(".lines.json", ""))

        train_count = round(len(new_list)/10*7)
        valid_count = round(len(new_list)/10*2)
        random.shuffle(new_list)
        train_set = new_list[:train_count]
        valid_set = new_list[train_count:train_count + valid_count]
        test_set = new_list[train_count + valid_count:]
        with open(f"{self.train_dir}train.txt", "w") as f:
            for i in train_set:
                shutil.copy(os.path.join(self.saved_dir, f"{i}.lines.json"), os.path.join(self.train_label_dir, f"{i}.lines.json"))
                shutil.copy(os.path.join(self.saved_dir, f"{i}.jpg"),os.path.join(self.train_img_dir, f"{i}.jpg"))
                f.write(f"images/{i}.jpg\n")

        with open(f"{self.valid_dir}valid.txt", "w") as f:
            for i in valid_set:
                shutil.copy(os.path.join(self.saved_dir, f"{i}.lines.json"),
                            os.path.join(self.valid_label_dir, f"{i}.lines.json"))
                shutil.copy(os.path.join(self.saved_dir, f"{i}.jpg"), os.path.join(self.valid_img_dir, f"{i}.jpg"))
                f.write(f"images/{i}.jpg\n")

        for i in test_set:
            shutil.copy(os.path.join(self.saved_dir, f"{i}.jpg"),os.path.join(self.test_img_dir, f"{i}.jpg"))

    def show_settings(self):
        dialog = SettingsDialog(self)
        dialog.auto_delete_checkbox.setChecked(self.auto_delete)
        if dialog.exec_():
            self.auto_delete = dialog.auto_delete_checkbox.isChecked()

    def log(self, message):
        self.log_box.append(message)

    def keyPressEvent(self, event):
        if event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_Z:
            self.revert_last_point()
        elif event.modifiers() == (Qt.ControlModifier | Qt.ShiftModifier) and event.key() == Qt.Key_Z:
            self.redo_last_point()


    def mouseMoveEvent(self, event):
        if not self.auto_track_enabled or not self.image_paths:
            return
        label_size = self.image_label.size()
        pixmap = QPixmap(self.image_paths[self.current_index])
        scaled_pixmap = pixmap.scaled(label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        x_offset = (label_size.width() - scaled_pixmap.width()) // 2
        y_offset = (label_size.height() - scaled_pixmap.height()) // 2
        x = event.pos().x() - x_offset
        y = event.pos().y() - y_offset
        if 0 <= x < scaled_pixmap.width() and 0 <= y < scaled_pixmap.height():
            x_ratio = pixmap.width() / scaled_pixmap.width()
            y_ratio = pixmap.height() / scaled_pixmap.height()
            real_x = x * x_ratio
            real_y = y * y_ratio
            point = QPointF(real_x, real_y)
            last = self.last_auto_point
            if last is None or ((point.x() - last.x())**2 + (point.y() - last.y())**2)**0.5 > 10:
                self.lanes[self.current_lane_idx].append(point)
                self.undo_stack[self.current_lane_idx].append(point)
                self.redo_stack[self.current_lane_idx].clear()
                self.last_auto_point = point
                self.show_image()

    def load_images(self):
        folder = QFileDialog.getExistingDirectory(self, "이미지 폴더 선택")
        if folder:
            self.image_paths = [
                os.path.join(folder, f)
                for f in sorted(os.listdir(folder))
                if f.lower().endswith(('.jpg', '.png', '.jpeg'))
            ]
            if not self.image_paths:
                QMessageBox.warning(self, "경고", "이미지가 없습니다.")
                return
            self.current_index = 0
            self.reset_all()
            self.show_image()

    def reset_all(self):
        self.lanes = [[]]
        self.undo_stack = [[]]
        self.redo_stack = [[]]
        self.current_lane_idx = 0
        self.last_auto_point = None

    def show_image(self):
        if not self.image_paths:
            return
        path = self.image_paths[self.current_index]
        original_pixmap = QPixmap(path)
        pixmap = QPixmap(original_pixmap)
        painter = QPainter(pixmap)
        for lane_idx, lane in enumerate(self.lanes):
            pen = QPen(COLORS[lane_idx % len(COLORS)], 10)
            painter.setPen(pen)
            for point in lane:
                painter.drawPoint(int(point.x()), int(point.y()))
        painter.end()
        self.image_label.setPixmap(pixmap.scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.list_widget.clear()
        for i, point in enumerate(self.lanes[self.current_lane_idx]):
            self.list_widget.addItem(f"[{self.current_lane_idx}] {i+1}: {point.x():.1f}, {point.y():.1f}")

    def show_prev(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.reset_all()
            self.show_image()

    def show_next(self):
        if self.auto_delete:
            self.save_labels()
            image_path = self.image_paths[self.current_index]
            if os.path.exists(image_path):
                os.remove(image_path)
        if self.current_index < len(self.image_paths) - 1:
            self.current_index += 1
            self.reset_all()
            self.show_image()

    def image_clicked(self, event):
        if not self.image_paths:
            return
        label_size = self.image_label.size()
        pixmap = QPixmap(self.image_paths[self.current_index])
        scaled_pixmap = pixmap.scaled(label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        x_offset = (label_size.width() - scaled_pixmap.width()) // 2
        y_offset = (label_size.height() - scaled_pixmap.height()) // 2
        x = event.pos().x() - x_offset
        y = event.pos().y() - y_offset
        if 0 <= x < scaled_pixmap.width() and 0 <= y < scaled_pixmap.height():
            x_ratio = pixmap.width() / scaled_pixmap.width()
            y_ratio = pixmap.height() / scaled_pixmap.height()
            real_x = x * x_ratio
            real_y = y * y_ratio
            point = QPointF(real_x, real_y)
            self.lanes[self.current_lane_idx].append(point)
            self.undo_stack[self.current_lane_idx].append(point)
            self.redo_stack[self.current_lane_idx].clear()
            self.show_image()

    def switch_lane(self):
        self.current_lane_idx += 1
        while len(self.lanes) <= self.current_lane_idx:
            self.lanes.append([])
            self.undo_stack.append([])
            self.redo_stack.append([])
        self.last_auto_point = None
        self.show_image()

    def reset_points(self):
        self.reset_all()
        self.show_image()

    def revert_last_point(self):
        if self.lanes and self.lanes[self.current_lane_idx]:
            point = self.lanes[self.current_lane_idx].pop()
            self.redo_stack[self.current_lane_idx].append(point)
            self.show_image()

    def redo_last_point(self):
        if self.redo_stack[self.current_lane_idx]:
            point = self.redo_stack[self.current_lane_idx].pop()
            self.lanes[self.current_lane_idx].append(point)
            self.show_image()

    def save_labels(self):
        if not self.image_paths:
            return
        image_path = self.image_paths[self.current_index]
        filename = os.path.basename(image_path)
        shutil.copy(image_path, os.path.join(self.saved_dir, filename))
        json_path = os.path.join(self.saved_dir, filename.rsplit('.', 1)[0] + "lines.json")
        label_data = {
            "Lines": [
                [{"x": f"{p.x():.1f}", "y": f"{p.y():.1f}"} for p in lane]
                for lane in self.lanes if lane
            ]
        }
        with open(json_path, "w") as f:
            json.dump(label_data, f, indent=4)
        original_pixmap = QPixmap(image_path)
        pixmap = QPixmap(original_pixmap)
        painter = QPainter(pixmap)
        for lane_idx, lane in enumerate(self.lanes):
            pen = QPen(COLORS[lane_idx % len(COLORS)], 10)
            painter.setPen(pen)
            for point in lane:
                painter.drawPoint(int(point.x()), int(point.y()))
        painter.end()
        pixmap.save(os.path.join(self.saved_dir, filename))
        self.log(f"저장 완료: {filename}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    tool = LabelTool()
    tool.show()
    sys.exit(app.exec_())