import cv2

def process_image(input_path, output_path):
    # Read the image
    image = cv2.imread(input_path)

    # Resize to 160x120 if needed
    if image.shape[1] != 160 or image.shape[0] != 120:
        image = cv2.resize(image, (160, 120))

    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Upscale to 640x440
    upscaled_image = cv2.resize(gray_image, (640, 440))

    # Write the output image
    cv2.imwrite(output_path, upscaled_image)

# Example usage
input_path = '/workspaces/deepracer-devel/output/virtual/dr-demo-2.jpg'
output_path = '/workspaces/deepracer-devel/output/virtual/dr-demo-2-resize.jpg'
process_image(input_path, output_path)