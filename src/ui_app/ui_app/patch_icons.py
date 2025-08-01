import re
import sys
from pathlib import Path

def patch_ui_resource_paths(file_path):
    file = Path(file_path)
    if not file.exists():
        print(f"❌ File not found: {file_path}")
        return

    lines = file.read_text(encoding='utf-8').splitlines()
    patched_lines = []

    # Regex to match QIcon.addFile(u"path/to/file")
    icon_pattern = re.compile(r'addFile\((u?")([^:/][^"]+)"')

    # Regex to match QPixmap("path/to/file")
    pixmap_pattern = re.compile(r'QPixmap\((u?")([^:/][^"]+)"')

    for line in lines:
        # Patch QIcon.addFile
        icon_match = icon_pattern.search(line)
        if icon_match:
            original_path = icon_match.group(2)
            fixed_path = f'":/{original_path}"'
            line = line.replace(f'"{original_path}"', fixed_path)

        # Patch QPixmap
        pixmap_match = pixmap_pattern.search(line)
        if pixmap_match:
            original_path = pixmap_match.group(2)
            fixed_path = f'":/{original_path}"'
            line = line.replace(f'"{original_path}"', fixed_path)

        patched_lines.append(line)

    file.write_text('\n'.join(patched_lines), encoding='utf-8')
    print(f"✅ Patched icon + pixmap paths in: {file_path}")

# Run with: python patch_qrc_paths.py ui_mainwindow.py
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python patch_qrc_paths.py <ui_file.py>")
    else:
        patch_ui_resource_paths(sys.argv[1])
