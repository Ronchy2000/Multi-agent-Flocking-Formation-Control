import os
from PIL import Image

def concatenate_images_grid(image_folder, output_path, prefix, total, rows=4, cols=2):
    """
    将指定文件夹中以prefix开头的前total张图片拼接成网格布局的大图

    参数：
    - image_folder: 图片所在文件夹路径
    - output_path: 输出拼接后图片的路径
    - prefix: 图片文件名前缀（如'trajectories_'）
    - total: 要拼接的图片数量
    - rows: 网格的行数
    - cols: 网格的列数
    """
    images = []
    for t in range(total):
        # 假设图片命名为'trajectories_00000.png', 'trajectories_00125.png', ..., 'trajectories_01000.png'
        # 根据实际命名规则调整此处
        iter_num = t * (1000 // total)  # 根据ITERATION=1000和SNAPSHOT_COUNT=8
        image_path = os.path.join(image_folder, f"{prefix}_{iter_num:05d}.png")
        if os.path.exists(image_path):
            img = Image.open(image_path)
            images.append(img)
        else:
            print(f"Warning: {image_path} does not exist.")

    if len(images) != total:
        print(f"Warning: Expected {total} images, but found {len(images)} images.")

    if not images:
        print("No images to concatenate.")
        return

    # 获取每张图片的尺寸（假设所有图片尺寸相同）
    img_width, img_height = images[0].size

    # 创建一个新的空白图片，白色背景
    new_im = Image.new('RGB', (cols * img_width, rows * img_height), (255, 255, 255))

    for index, img in enumerate(images):
        row = index // cols
        col = index % cols
        if row < rows and col < cols:
            new_im.paste(img, (col * img_width, row * img_height))
        else:
            print(f"Skipping image {index} as it exceeds the grid size.")

    # 保存拼接后的图片
    new_im.save(output_path)
    print(f"Saved concatenated image to {output_path}")

if __name__ == "__main__":
    # 设置参数
    image_folder = "snapshots"  # 图片所在文件夹
    output_path = os.path.join(image_folder, "concatenated_trajectories.png")  # 输出图片路径
    prefix = "trajectories"  # 图片前缀
    total = 8  # 要拼接的图片数量
    rows = 4  # 行数
    cols = 2  # 列数

    concatenate_images_grid(image_folder, output_path, prefix, total, rows, cols)
