"""Camera OCR for road-sign text (Tesseract), the text front-end of the sign-reading head.

llava-phi3 sees the camera image at 336 px, where a 2.6 m sign 15 m away is ~25 px wide and its words
are unreadable. Tesseract reads the full-resolution frame instead. Signs stand at the roadside near the
horizon, so only the horizon band is read, upscaled 2x and binarised. If Tesseract is not installed the
reader is disabled and returns nothing (the VLA then relies on its own `signs` field only).
"""
import re

import numpy as np

try:
    import cv2
    import pytesseract
    pytesseract.get_tesseract_version()
    AVAILABLE = True
except Exception:  # missing binary or module
    AVAILABLE = False

_CFG = '--psm 11 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ'


def read_sign_text(frame_bgr, band=(0.25, 0.65), right=0.4, scale=3.0, min_len=3):
    """Return the words read on signs (upper case, >= min_len letters), joined in reading order.
    Only the horizon band of the right-hand roadside (x > right * width) is read: roadside signs stand there.
    Two binarisations (global Otsu, local adaptive) are read and merged; each misses some sizes."""
    if not AVAILABLE or frame_bgr is None:
        return ''
    h, w = frame_bgr.shape[:2]
    crop = frame_bgr[int(band[0] * h):int(band[1] * h), int(right * w):]
    g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    _, otsu = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    adapt = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 10)
    words = []
    for img in (otsu, adapt):
        txt = pytesseract.image_to_string(img, config=_CFG)
        words += [x for x in re.findall(r'[A-Z]+', txt.upper()) if len(x) >= min_len]
    return ' '.join(dict.fromkeys(words))


def _selftest():
    """Render a sign-like image and read it back (used by the tests when Tesseract is present)."""
    img = np.full((400, 800, 3), 255, np.uint8)
    cv2.putText(img, 'ROAD WORK', (180, 200), cv2.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 0), 6)
    return read_sign_text(img, band=(0.0, 1.0), right=0.0)
