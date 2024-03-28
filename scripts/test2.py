import cv2

# Load the image
image = cv2.imread('/home/robotics/Downloads/kccs0217/saved_data/image/1708213933663243949.jpg')
image = cv2.resize(image, (448, 448))

start_point = (281, 228)  # Top left corner of the rectangle
end_point = (295, 278)    # Bottom right corner of the rectangle
color = (0, 255, 0)  # Green in BGR
thickness = 2        # Thickness of the rectangle border in pixels

# Draw the rectangle on the image
image_with_rectangle = cv2.rectangle(image, start_point, end_point, color, thickness)

# Display the image with the rectangle
cv2.imshow('Image with Rectangle', image_with_rectangle)
cv2.waitKey(0)
cv2.destroyAllWindows()
