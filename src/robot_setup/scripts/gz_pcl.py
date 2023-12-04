import pcl
import os
import lxml.etree as ET

def create_sphere_xml(model_name, position, radius=0.05):
    """ Generates XML for a sphere model in Gazebo. """
    model = ET.Element("model", name=model_name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{position[0]} {position[1]} {position[2]} 0 0 0"

    link = ET.SubElement(model, "link", name="link")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = str(radius)

    return model

# Load Point Cloud
cloud = pcl.load_XYZRGB('/home/zhang/Downloads/Elephant_1.pcd') # Update path
points = cloud.to_array()

# Create a Gazebo World
world = ET.Element("sdf", version="1.6")
world_model = ET.SubElement(world, "world", name="default")

# Add each point as a sphere in the world
for i, point in enumerate(points):
    # Assuming point is [x, y, z, rgb]
    sphere_xml = create_sphere_xml(f"point_{i}", point[:3])
    world_model.append(sphere_xml)

# Write to file
tree = ET.ElementTree(world)
tree.write("point_cloud_world.world", pretty_print=True, xml_declaration=True, encoding="utf-8")

print("Gazebo world file generated: point_cloud_world.world")
