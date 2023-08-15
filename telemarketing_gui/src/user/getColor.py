import cv2

img = cv2.imread("src/user/images/prueba.png")
cv2.imshow("Image",img)

print(img[0])

# waitKey() waits for a key press to close the window and 0 specifies indefinite loop
cv2.waitKey(0)
 
# cv2.destroyAllWindows() simply destroys all the windows we created.
cv2.destroyAllWindows()