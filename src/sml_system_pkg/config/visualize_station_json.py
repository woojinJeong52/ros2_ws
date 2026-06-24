#!/usr/bin/env python3
# visualize_station_json.py

import argparse
import json
import matplotlib.pyplot as plt


def load_json(json_path: str) -> dict:
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)


def extract_points(data: dict):
    """
    지원 JSON 구조:

    {
      "map": {
        "name": "A_zone_estimated",
        "width_m": 10.0,
        "height_m": 7.5
      },
      "station_coordinates": {
        "0": {
          "name": "A_START_GOAL",
          "x": 3.99,
          "y": 1.12
        }
      }
    }
    """

    if "station_coordinates" not in data:
        raise ValueError("JSON 안에 station_coordinates 키가 없습니다.")

    map_info = data.get("map", {})

    map_name = map_info.get("name", "station_map")
    width_m = float(map_info.get("width_m", 10.0))
    height_m = float(map_info.get("height_m", 7.5))

    points = []

    for station_id, info in data["station_coordinates"].items():
        points.append({
            "id": int(station_id),
            "name": info.get("name", f"station_{station_id}"),
            "x": float(info["x"]),
            "y": float(info["y"]),
        })

    points.sort(key=lambda p: p["id"])

    return map_name, width_m, height_m, points


def visualize(map_name, width_m, height_m, points, save_path=None, invert_y=False):
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]

    plt.figure(figsize=(10, 7.5))

    plt.scatter(xs, ys, s=90)

    for p in points:
        label = f'{p["id"]}: {p["name"]}'
        plt.annotate(
            label,
            (p["x"], p["y"]),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=9,
        )

    plt.title(f"{map_name} - Station Coordinates")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    plt.xlim(0, width_m)
    plt.ylim(0, height_m)

    plt.xticks(range(0, int(width_m) + 1, 1))
    plt.yticks([i * 0.5 for i in range(0, int(height_m * 2) + 1)])

    plt.grid(True)
    plt.gca().set_aspect("equal", adjustable="box")

    if invert_y:
        plt.gca().invert_yaxis()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[SAVE] {save_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize station coordinates from JSON file"
    )

    parser.add_argument(
        "--json",
        required=True,
        help="station_coordinates JSON 파일 경로",
    )

    parser.add_argument(
        "--save",
        default=None,
        help="이미지 저장 경로. 예: a_zone_station_map.png",
    )

    parser.add_argument(
        "--invert-y",
        action="store_true",
        help="이미지 좌표계처럼 y축을 뒤집어서 보고 싶을 때 사용",
    )

    args = parser.parse_args()

    data = load_json(args.json)
    map_name, width_m, height_m, points = extract_points(data)

    print(f"[MAP] {map_name}")
    print(f"[SIZE] width={width_m} m, height={height_m} m")
    print()

    for p in points:
        print(
            f'{p["id"]:>2} | {p["name"]:<25} | '
            f'x={p["x"]:.3f}, y={p["y"]:.3f}'
        )

    visualize(
        map_name=map_name,
        width_m=width_m,
        height_m=height_m,
        points=points,
        save_path=args.save,
        invert_y=args.invert_y,
    )


if __name__ == "__main__":
    main()