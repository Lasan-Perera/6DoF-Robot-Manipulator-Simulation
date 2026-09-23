import sys
import json
from transformers import AutoModelForCausalLM
from PIL import Image

sys.path.insert(0, "../")  # gp7_pick_place/ -- for pixel_to_world
import pixel_to_world as p2w

query = sys.argv[1]
top_z = float(sys.argv[2])

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
with open("target.json", "w") as f:
    json.dump(target, f, indent=2)

print(f"'{query}' -> pixel ({pt['x']:.3f}, {pt['y']:.3f}) -> world ({world_x:.4f}, {world_y:.4f})")
print("saved target.json")
