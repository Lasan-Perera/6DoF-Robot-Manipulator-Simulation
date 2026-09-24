"""
Query Moondream for an object's location in the last captured frame,
convert to world (x,y) via pixel_to_world, save as <output_name>.json
for the ROS-side script to pick up.

Usage: python3 locate_object.py "the red box" 0.05 pick_target
       python3 locate_object.py "the green platform" 0.05 place_target
       (second arg = known top-surface z of that object,
        third arg = output filename, defaults to "target")
"""
import sys
import json
from transformers import AutoModelForCausalLM
from PIL import Image

sys.path.insert(0, "../")  # gp7_pick_place/ -- for pixel_to_world
import pixel_to_world as p2w

query = sys.argv[1]
top_z = float(sys.argv[2])
output_name = sys.argv[3] if len(sys.argv) > 3 else "target"

model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2", revision="2025-04-14", trust_remote_code=True,
)
img = Image.open("frame.png")

result = model.point(img, query)
if not result["points"]:
    print(f"Moondream found no match for '{query}'")
    sys.exit(1)

pt = result["points"][0]
world_x, world_y = p2w.pixel_to_world(pt["x"], pt["y"], top_z)

target = {"query": query, "pixel": pt, "world_x": world_x, "world_y": world_y, "top_z": top_z}
outfile = f"{output_name}.json"
with open(outfile, "w") as f:
    json.dump(target, f, indent=2)

print(f"'{query}' -> pixel ({pt['x']:.3f}, {pt['y']:.3f}) -> world ({world_x:.4f}, {world_y:.4f})")
print(f"saved {outfile}")
