from PIL import Image
import os

def create_montage(snapshot_dir, output_path, grid_size=(4, 2), image_size=(800, 800)):
    """
    将快照图拼接成一个大的蒙太奇图像。
    
    参数：
    - snapshot_dir: 快照图所在的目录
    - output_path: 输出的拼接图像路径
    - grid_size: 拼接网格的行数和列数，例如 (4, 2) 表示4行2列
    - image_size: 每张快照图的尺寸，必须与生成的快照图尺寸一致
    """
    rows, cols = grid_size
    montage_width = cols * image_size[0]
    montage_height = rows * image_size[1]
    montage_image = Image.new('RGB', (montage_width, montage_height), (255, 255, 255))

    snapshot_files = sorted([f for f in os.listdir(snapshot_dir) if f.startswith("snapshot_") and f.endswith(".png")])

    for idx, file_name in enumerate(snapshot_files):
        if idx >= rows * cols:
            break
        img_path = os.path.join(snapshot_dir, file_name)
        img = Image.open(img_path)
        img = img.resize(image_size)  # 确保尺寸一致
        row = idx // cols
        col = idx % cols
        montage_image.paste(img, (col * image_size[0], row * image_size[1]))

    montage_image.save(output_path)
    print(f"Montage image saved to {output_path}")

# 使用示例
if __name__ == "__main__":
    snapshot_directory = "snapshots"
    output_montage = "montage.png"
    create_montage(snapshot_directory, output_montage, grid_size=(4, 2), image_size=(800, 800))
