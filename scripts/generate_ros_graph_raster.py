from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT_DIR = Path(r"C:\Users\chen yin\Desktop\lunwen")
JPEG_PATH = OUT_DIR / "ROS节点通信图_论文版.jpeg"
TIFF_PATH = OUT_DIR / "ROS节点通信图_论文版.tiff"

WIDTH = 2400
HEIGHT = 1500
BG = (250, 251, 252)
NODE_FILL = (226, 239, 255)
NODE_OUTLINE = (37, 99, 235)
TOPIC_FILL = (254, 243, 199)
TOPIC_OUTLINE = (217, 119, 6)
TEXT = (15, 23, 42)
SUBTEXT = (71, 85, 105)
EDGE = (71, 85, 105)


def load_font(size, bold=False):
    candidates = [
        r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


FONT_TITLE = load_font(42, bold=True)
FONT_SUBTITLE = load_font(24)
FONT_NODE = load_font(30, bold=True)
FONT_TOPIC = load_font(24, bold=True)
FONT_DESC = load_font(22)


def draw_centered_text(draw, box, text, font, fill):
    left, top, right, bottom = box
    bbox = draw.multiline_textbbox((0, 0), text, font=font, align="center", spacing=4)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = left + (right - left - w) / 2
    y = top + (bottom - top - h) / 2 - 2
    draw.multiline_text((x, y), text, font=font, fill=fill, align="center", spacing=4)


def draw_arrow(draw, start, end, color=EDGE, width=4):
    sx, sy = start
    ex, ey = end
    draw.line((sx, sy, ex, ey), fill=color, width=width)
    import math

    angle = math.atan2(ey - sy, ex - sx)
    size = 16
    a1 = angle + math.pi * 0.85
    a2 = angle - math.pi * 0.85
    p1 = (ex + size * math.cos(a1), ey + size * math.sin(a1))
    p2 = (ex + size * math.cos(a2), ey + size * math.sin(a2))
    draw.polygon([end, p1, p2], fill=color)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img)

    draw.text((80, 60), "图 4-3  ROS 节点通信关系示意图", font=FONT_TITLE, fill=TEXT)
    draw.text(
        (80, 118),
        "说明：矩形表示 ROS 节点，椭圆表示话题，箭头表示消息发布方向；布局风格参考 rqt_graph 并按论文插图规范整理。",
        font=FONT_SUBTITLE,
        fill=SUBTEXT,
    )

    nodes = {
        "mission_ui": (90, 250, 390, 350),
        "avoidance_2d": (90, 1030, 420, 1130),
        "swarm_manager": (980, 1030, 1320, 1130),
        "formation_controller": (1820, 1030, 2300, 1130),
    }

    topics = {
        "/swarm/formation_cmd": (520, 210, 930, 290),
        "/swarm/obstacles": (555, 340, 890, 420),
        "/swarm/mission_anchor": (505, 470, 945, 550),
        "/swarm/mission_phase_cmd": (470, 600, 980, 680),
        "/swarm/formation_cmd_safe": (1080, 210, 1590, 290),
        "/swarm/avoidance/state": (1120, 340, 1550, 420),
        "/swarm/status": (1165, 470, 1505, 550),
        "/swarm/debug_anchor": (1545, 210, 2025, 290),
        "/swarm/formation_markers": (1480, 340, 2090, 420),
        "/swarm/gazebo_sync_status": (1450, 470, 2120, 550),
    }

    for name, box in nodes.items():
        draw.rounded_rectangle(box, radius=18, fill=NODE_FILL, outline=NODE_OUTLINE, width=4)
        draw_centered_text(draw, box, name, FONT_NODE, TEXT)

    for name, box in topics.items():
        draw.ellipse(box, fill=TOPIC_FILL, outline=TOPIC_OUTLINE, width=4)
        draw_centered_text(draw, box, name, FONT_TOPIC, (120, 53, 15))

    # mission_ui publishers
    draw_arrow(draw, (390, 300), (520, 250))
    draw_arrow(draw, (390, 305), (555, 380))
    draw_arrow(draw, (390, 315), (505, 510))
    draw_arrow(draw, (390, 325), (470, 640))

    # avoidance subscriptions and publications
    draw_arrow(draw, (720, 290), (230, 1030))
    draw_arrow(draw, (720, 420), (290, 1030))
    draw_arrow(draw, (420, 1080), (1080, 250))
    draw_arrow(draw, (420, 1095), (1120, 380))

    # swarm_manager subscriptions/publications
    draw_arrow(draw, (725, 680), (1120, 1030))
    draw_arrow(draw, (1335, 290), (1140, 1030))
    draw_arrow(draw, (1320, 1080), (1165, 510))

    # formation_controller subscriptions/publications
    draw_arrow(draw, (1335, 290), (1980, 1030))
    draw_arrow(draw, (930, 250), (1910, 1030))
    draw_arrow(draw, (2060, 1080), (1545, 250))
    draw_arrow(draw, (2060, 1090), (1480, 380))
    draw_arrow(draw, (2060, 1100), (1450, 510))

    # mission_ui subscriptions
    draw_arrow(draw, (1165, 510), (390, 340))
    draw_arrow(draw, (1120, 380), (390, 335))
    draw_arrow(draw, (1165, 510), (390, 330))
    draw_arrow(draw, (1450, 510), (390, 345))

    # captions under columns
    draw.text((120, 930), "人机交互与场景配置", font=FONT_DESC, fill=SUBTEXT)
    draw.text((1020, 930), "状态管理", font=FONT_DESC, fill=SUBTEXT)
    draw.text((1910, 930), "编队控制与可视化", font=FONT_DESC, fill=SUBTEXT)

    # explanatory notes
    notes_y = 1210
    notes = [
        "1. mission_ui 负责发布编队命令、障碍物信息、任务锚点和任务阶段信息，并订阅系统状态与实验摘要相关话题。",
        "2. avoidance_2d 根据 /swarm/formation_cmd 与 /swarm/obstacles 生成安全编队命令 /swarm/formation_cmd_safe。",
        "3. swarm_manager 监听安全编队与任务阶段并统一发布 /swarm/status，用于界面侧状态显示。",
        "4. formation_controller 同时监听原始与安全编队命令，输出调试锚点、编队标记及 Gazebo 同步状态。",
    ]
    for i, note in enumerate(notes):
        draw.text((90, notes_y + i * 48), note, font=FONT_DESC, fill=TEXT)

    img.save(JPEG_PATH, format="JPEG", quality=95, subsampling=0)
    img.save(TIFF_PATH, format="TIFF")
    print(f"Saved: {JPEG_PATH}")
    print(f"Saved: {TIFF_PATH}")


if __name__ == "__main__":
    main()
