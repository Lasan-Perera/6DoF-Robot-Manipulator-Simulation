"""
Query Moondream once for every (pick, place) pair in SEQUENCE, save all
results to sequence_targets.json. Run this in vla_venv, AFTER capturing
a frame (capture_frame.py) -- no rclpy needed here.
"""
import json
import sys
sys.path.insert(0, "../")  # gp7_pick_place/ -- for pixel_to_world
from transformers import AutoModelForCausalLM
from PIL import Image
import pixel_to_world as p2w

OBJECT_TOP_Z = 0.05

SEQUENCE = [
    ("the small red cube",   "the wide flat red square"),
    ("the small blue cube",  "the wide flat blue square"),
    ("the small green cube", "the wide flat green square"),
]

model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2", revision="2025-04-14", trust_remote_code=True,
)
img = Image.open("frame.png")


def locate(query):
    result = model.point(img, query)
    if not result["points"]:
        raise RuntimeError(f"Moondream found no match for '{query}'")
    pt = result["points"][0]
    wx, wy = p2w.pixel_to_world(pt["x"], pt["y"], OBJECT_TOP_Z)
    print(f"'{query}' -> pixel ({pt['x']:.3f}, {pt['y']:.3f}) -> world ({wx:.4f}, {wy:.4f})")
    return {"query": query, "world_x": wx, "world_y": wy, "top_z": OBJECT_TOP_Z}


results = []
for pick_query, place_query in SEQUENCE:
    print(f"\n--- {pick_query} -> {place_query} ---")
    pick = locate(pick_query)
    place = locate(place_query)
    results.append({"pick": pick, "place": place})

with open("sequence_targets.json", "w") as f:
    json.dump(results, f, indent=2)

print(f"\nsaved sequence_targets.json ({len(results)} rounds)")
