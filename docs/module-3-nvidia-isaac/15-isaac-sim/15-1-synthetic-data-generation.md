# Lesson 15.1: Synthetic Data Generation

Training a modern deep learning model for a task like object detection requires a huge amount of labeled data. For example, the COCO dataset, a common benchmark, has over 200,000 labeled images. Creating such a dataset by hand—taking pictures and then manually drawing bounding boxes around every object—is incredibly slow and expensive.

**Synthetic data** solves this problem by generating the images and the labels automatically inside a simulator.

## Isaac Sim Replicator

Isaac Sim's tool for this is called **Replicator**. Replicator is a Python scripting interface that allows you to define how to generate a dataset.

A typical Replicator script does the following:
1.  **Define the environment:** Places the objects, walls, and lights in the scene.
2.  **Define the randomization:** Specifies how to vary the scene for each image (e.g., move the objects around, change the lighting). This is the "Domain Randomization" we will cover in the next lesson.
3.  **Attach annotators:** Specifies what kind of labels to generate for each image.
4.  **Run the replicator:** Tells Isaac Sim to generate a specified number of images and their corresponding labels.

## Annotators: The "Perfect" Labels

Because we are in a simulation, we have perfect knowledge of everything in the scene. We know the exact 3D position and class of every object. The Replicator framework uses "annotators" to convert this ground-truth information into the labels that a machine learning model needs.

Common annotators include:
*   **Bounding Box Annotator:** Generates 2D or 3D bounding boxes around each object in the image.
*   **Semantic Segmentation Annotator:** Generates an image where each pixel is colored based on the class of the object it belongs to (e.g., all cars are blue, all pedestrians are red).
*   **Instance Segmentation Annotator:** Generates an image where each pixel is colored based on the specific *instance* of the object (e.g., car_1 is blue, car_2 is green).
*   **Depth Annotator:** Generates a depth image.
*   **Pose Annotator:** Generates the exact 6D pose (position and orientation) of each object.

## A Simple Replicator Script

Here is a simplified example of what a Replicator script might look like.

```python
import omni.replicator.core as rep

# Define our objects
sphere = rep.create.sphere(position=(0, 0, 1))
cube = rep.create.cube(position=(2, 0, 1))

# Set up the camera and renderer
camera = rep.create.camera()
render_product = rep.create.render_product(camera, (1024, 1024))

# Attach annotators to generate bounding box labels
bbox_annotator = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
bbox_annotator.attach([render_product])

# Define a randomization function for the sphere's position
def randomize_sphere():
    with sphere:
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 1), (2, 2, 1))
        )
    return sphere

# Register the randomization function to be called for each frame
rep.randomizer.register(randomize_sphere)

# Run the replicator for 100 frames
with rep.trigger.on_frame(num_frames=100):
    rep.randomizer.randomize()

# The script would then be run inside Isaac Sim, which would output
# 100 images and 100 corresponding text files with the bounding box data.
```

This ability to generate vast, clean, and perfectly-labeled datasets is the primary reason why high-fidelity simulators like Isaac Sim are at the heart of modern AI-based robotics. It dramatically reduces the time and cost required to train robust and accurate perception models.
