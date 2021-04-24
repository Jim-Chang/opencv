import cv2

RED_COLOR = (200, 58, 76)
WHITE_COLOR = (255, 255, 255)

# img => RGB
def draw_locations(img, match_results, scale=1):
    for match_result in match_results:
        y1, x2, y2, x1 = match_result.location
        y1, x2, y2, x1 = y1 * scale, x2 * scale, y2 * scale, x1 * scale
        cv2.rectangle(img, (x1, y1), (x2, y2), RED_COLOR, 2)
        cv2.rectangle(img, (x1, y2 - 35), (x2, y2), RED_COLOR, cv2.FILLED)
        cv2.putText(img, match_result.name, (x1 + 10, y2-10), cv2.FONT_HERSHEY_COMPLEX, 0.8, WHITE_COLOR, 2)

# img => RGB
def show_to_window(img, window='cam', wait=1):
    cv2.imshow(window, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    cv2.waitKey(wait)