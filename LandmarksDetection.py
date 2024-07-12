import face_alignment
from skimage import io

fa = face_alignment.FaceAlignment(face_alignment.LandmarksType, flip_input=False)

input = io.imread('../data/image_1.png')
preds = fa.get_landmarks(input)

print(preds)