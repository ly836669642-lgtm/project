# Traffic light state detected from the RGB camera (HSV blob analysis).
std_msgs/Header header

uint8 NONE=0     # no light facing us detected
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3
uint8 state

uint32 pixel_count  # area of the winning lamp blob [px]
float32 cx          # blob center, normalized [0..1] image coordinates
float32 cy
