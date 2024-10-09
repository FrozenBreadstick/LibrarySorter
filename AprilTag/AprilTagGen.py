import cv2
import apriltag

tag_family = 'tag36h11'  # Choose the tag family
tags_to_generate = 100    # Number of tags you want

detector = apriltag.TagDetector()
for tag_id in range(tags_to_generate):
    tag = detector.create(tag_id, tag_family)
    img = tag.render()  # Create the tag image
    cv2.imwrite(f'apriltag_{tag_id}.png', img)