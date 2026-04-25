from pathlib import Path


OUT_PATH = Path(r"C:\Users\chen yin\Desktop\lunwen\ROS节点通信图.svg")


SVG = """<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="1800" height="1100" viewBox="0 0 1800 1100">
  <defs>
    <style>
      .bg { fill: #f8fafc; }
      .node { fill: #dbeafe; stroke: #2563eb; stroke-width: 3; rx: 16; ry: 16; }
      .nodeText { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; font-size: 28px; fill: #0f172a; font-weight: 700; }
      .topic { fill: #fef3c7; stroke: #d97706; stroke-width: 3; }
      .topicText { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; font-size: 21px; fill: #78350f; font-weight: 600; }
      .desc { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; font-size: 22px; fill: #334155; }
      .edge { stroke: #475569; stroke-width: 3.5; fill: none; marker-end: url(#arrow); }
      .title { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; font-size: 38px; fill: #0f172a; font-weight: 800; }
      .subtitle { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; font-size: 22px; fill: #475569; }
    </style>
    <marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
      <path d="M0,0 L12,6 L0,12 z" fill="#475569"/>
    </marker>
  </defs>

  <rect class="bg" x="0" y="0" width="1800" height="1100"/>
  <text class="title" x="70" y="70">ROS 节点通信关系图</text>
  <text class="subtitle" x="70" y="108">节点采用矩形表示，话题采用椭圆表示，箭头方向表示消息发布流向。整体风格参考 rqt_graph。</text>

  <rect class="node" x="80" y="200" width="260" height="90"/>
  <text class="nodeText" x="145" y="255">mission_ui</text>

  <rect class="node" x="80" y="820" width="280" height="90"/>
  <text class="nodeText" x="125" y="875">avoidance_2d</text>

  <rect class="node" x="760" y="820" width="300" height="90"/>
  <text class="nodeText" x="810" y="875">swarm_manager</text>

  <rect class="node" x="1410" y="820" width="330" height="90"/>
  <text class="nodeText" x="1452" y="875">formation_controller</text>

  <ellipse class="topic" cx="520" cy="180" rx="150" ry="42"/>
  <text class="topicText" x="420" y="188">/swarm/formation_cmd</text>

  <ellipse class="topic" cx="520" cy="300" rx="128" ry="42"/>
  <text class="topicText" x="433" y="308">/swarm/obstacles</text>

  <ellipse class="topic" cx="520" cy="420" rx="165" ry="42"/>
  <text class="topicText" x="406" y="428">/swarm/mission_anchor</text>

  <ellipse class="topic" cx="520" cy="540" rx="178" ry="42"/>
  <text class="topicText" x="394" y="548">/swarm/mission_phase_cmd</text>

  <ellipse class="topic" cx="900" cy="180" rx="188" ry="42"/>
  <text class="topicText" x="770" y="188">/swarm/formation_cmd_safe</text>

  <ellipse class="topic" cx="900" cy="300" rx="168" ry="42"/>
  <text class="topicText" x="792" y="308">/swarm/avoidance/state</text>

  <ellipse class="topic" cx="900" cy="420" rx="118" ry="42"/>
  <text class="topicText" x="825" y="428">/swarm/status</text>

  <ellipse class="topic" cx="1290" cy="180" rx="170" ry="42"/>
  <text class="topicText" x="1176" y="188">/swarm/debug_anchor</text>

  <ellipse class="topic" cx="1290" cy="300" rx="195" ry="42"/>
  <text class="topicText" x="1154" y="308">/swarm/formation_markers</text>

  <ellipse class="topic" cx="1290" cy="420" rx="192" ry="42"/>
  <text class="topicText" x="1152" y="428">/swarm/gazebo_sync_status</text>

  <path class="edge" d="M340 245 C380 220, 390 205, 370 190"/>
  <path class="edge" d="M340 245 C410 255, 410 285, 390 295"/>
  <path class="edge" d="M340 245 C410 320, 410 395, 370 410"/>
  <path class="edge" d="M340 245 C430 390, 430 505, 360 530"/>

  <path class="edge" d="M520 222 C520 420, 210 560, 210 815"/>
  <path class="edge" d="M520 342 C520 520, 250 610, 250 815"/>

  <path class="edge" d="M360 865 C470 865, 560 600, 780 210"/>
  <path class="edge" d="M360 855 C540 855, 560 640, 770 310"/>

  <path class="edge" d="M520 582 C520 760, 850 760, 850 815"/>
  <path class="edge" d="M900 222 C900 760, 910 760, 910 815"/>

  <path class="edge" d="M1060 855 C1180 855, 1430 630, 1440 210"/>
  <path class="edge" d="M1080 865 C1210 865, 1450 680, 1450 320"/>
  <path class="edge" d="M1080 875 C1210 875, 1455 735, 1460 430"/>

  <path class="edge" d="M760 420 C650 420, 650 420, 690 420"/>
  <path class="edge" d="M1020 420 C1110 420, 1110 420, 1095 420"/>
  <path class="edge" d="M1098 180 C1170 180, 1170 180, 1120 180"/>
  <path class="edge" d="M1098 300 C1170 300, 1170 300, 1120 300"/>
  <path class="edge" d="M1098 420 C1170 420, 1170 420, 1120 420"/>

  <text class="desc" x="70" y="980">说明 1：mission_ui 负责场景配置、编队命令发布、任务阶段切换及实验评估摘要刷新。</text>
  <text class="desc" x="70" y="1018">说明 2：avoidance_2d 依据障碍信息对编队锚点进行安全修正，并输出 /swarm/formation_cmd_safe 与避障状态。</text>
  <text class="desc" x="70" y="1056">说明 3：swarm_manager 监听安全编队与任务阶段，统一广播 /swarm/status；formation_controller 计算目标队形并发布调试与可视化话题。</text>
</svg>
"""


def main():
    OUT_PATH.write_text(SVG, encoding="utf-8")
    print(f"Saved: {OUT_PATH}")


if __name__ == "__main__":
    main()
